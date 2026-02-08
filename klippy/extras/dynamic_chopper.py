# Dynamic Chopper Control (DCC) - Phase 1 Python Prototype
# Velocity-synchronized TMC5160 register modulation for Kalico
#
# Reads planned velocity profiles from the motion pipeline and schedules
# timed SPI register writes so chopper parameters are always optimal for
# the current operating conditions. Think "input shaping but for TMC
# driver registers."
#
# Copyright (C) 2026 GM TC / Günter Michaeller Technical Consulting
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging

from . import tmc


class DynamicChopper:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.stepper_name = config.get_name().split()[1]
        self.enabled = config.getboolean('enable', True)
        # Velocity zone boundaries (sorted ascending, mm/s)
        zone_vel_str = config.get('zone_velocities')
        self.zone_velocities = sorted(
            float(v.strip()) for v in zone_vel_str.split(','))
        self.num_zones = len(self.zone_velocities) + 1
        # TMC register fields that DCC modulates per zone
        self._dcc_fields = [
            # CHOPCONF
            'toff', 'hstrt', 'hend', 'tbl', 'tpfd', 'chm',
            'vhighfs', 'vhighchm',
            # PWMCONF
            'pwm_ofs', 'pwm_grad', 'pwm_freq', 'pwm_autoscale',
            'pwm_autograd', 'pwm_reg', 'pwm_lim',
            # GCONF
            'en_pwm_mode',
        ]
        # Parse per-zone field overrides from config
        self._zone_overrides = []
        for i in range(self.num_zones):
            overrides = {}
            for field in self._dcc_fields:
                val = config.getint('zone_%d_%s' % (i, field), None)
                if val is not None:
                    overrides[field] = val
            self._zone_overrides.append(overrides)
        # Reversal profile
        self._reversal_overrides = {}
        for field in self._dcc_fields:
            val = config.getint('reversal_%s' % field, None)
            if val is not None:
                self._reversal_overrides[field] = val
        self.reversal_current_scale = config.getfloat(
            'reversal_current_scale', None, minval=0.5, maxval=1.0)
        self.reversal_lead_time = config.getfloat(
            'reversal_lead_time', 0.0005)  # seconds before zero-crossing
        self.reversal_hold_time = config.getfloat(
            'reversal_hold_time', 0.001)   # seconds after zero-crossing
        # SPI lead time (seconds before zone crossing to send write)
        self.spi_lead_time = config.getfloat('spi_lead_time', 0.00001)
        # Runtime state
        self.current_zone = -1
        self.prev_axes_r = None
        self.in_reversal = False
        self.toolhead = None
        self.mcu_tmc = None
        self.fields = None
        self.zone_registers = []
        self.reversal_registers = {}
        self.reversal_irun_regs = None
        # Events
        self.printer.register_event_handler(
            'klippy:connect', self._handle_connect)
        # G-code commands
        gcode = self.printer.lookup_object('gcode')
        gcode.register_mux_command(
            'DYNAMIC_CHOPPER_ENABLE', 'STEPPER', self.stepper_name,
            self.cmd_ENABLE, desc='Enable dynamic chopper control')
        gcode.register_mux_command(
            'DYNAMIC_CHOPPER_DISABLE', 'STEPPER', self.stepper_name,
            self.cmd_DISABLE, desc='Disable dynamic chopper control')
        gcode.register_mux_command(
            'DYNAMIC_CHOPPER_STATUS', 'STEPPER', self.stepper_name,
            self.cmd_STATUS, desc='Report dynamic chopper status')

    def _handle_connect(self):
        self.toolhead = self.printer.lookup_object('toolhead')
        tmc_obj = self.printer.lookup_object('tmc5160 ' + self.stepper_name)
        self.mcu_tmc = tmc_obj.mcu_tmc
        self.fields = self.mcu_tmc.get_fields()
        self._precompute_registers()
        self._precompute_reversal_current()
        self._hook_move_pipeline()
        logging.info("DCC: initialized %s — %d velocity zones, "
                     "boundaries at %s mm/s",
                     self.stepper_name, self.num_zones,
                     ', '.join('%.1f' % v for v in self.zone_velocities))

    # ------------------------------------------------------------------
    # Register pre-computation
    # ------------------------------------------------------------------

    def _precompute_registers(self):
        """Build full register values for each zone and reversal profile.

        We snapshot the base register values at connect time and apply
        our field overrides on top. This avoids touching the shared
        FieldHelper state during printing.
        """
        base = dict(self.fields.registers)
        for overrides in self._zone_overrides:
            self.zone_registers.append(
                self._build_reg_dict(base, overrides))
        self.reversal_registers = self._build_reg_dict(
            base, self._reversal_overrides)

    def _precompute_reversal_current(self):
        """Pre-compute IHOLD_IRUN register value with scaled irun for
        reversal current reduction. Also stores the normal value for
        restoration after reversal.
        """
        if self.reversal_current_scale is None:
            return
        base_ihold_irun = self.fields.registers.get('IHOLD_IRUN', 0)
        irun_mask = self.fields.all_fields['IHOLD_IRUN']['irun']
        irun = (base_ihold_irun & irun_mask) >> tmc.ffs(irun_mask)
        scaled_irun = int(irun * self.reversal_current_scale)
        scaled_irun = max(0, min(31, scaled_irun))
        rev_val = (base_ihold_irun & ~irun_mask) | (
            (scaled_irun << tmc.ffs(irun_mask)) & irun_mask)
        self.reversal_irun_regs = {
            'IHOLD_IRUN': rev_val,
        }
        self._normal_ihold_irun = base_ihold_irun
        logging.info("DCC %s: reversal current scale %.2f — "
                     "irun %d → %d",
                     self.stepper_name, self.reversal_current_scale,
                     irun, scaled_irun)

    def _build_reg_dict(self, base_regs, field_overrides):
        """Return {reg_name: value} with field_overrides applied on top."""
        regs = {}
        for field_name, value in field_overrides.items():
            reg_name = self.fields.field_to_register.get(field_name)
            if reg_name is None:
                continue
            if reg_name not in regs:
                regs[reg_name] = base_regs.get(reg_name, 0)
            mask = self.fields.all_fields[reg_name][field_name]
            regs[reg_name] = (
                (regs[reg_name] & ~mask) | ((value << tmc.ffs(mask)) & mask))
        return regs

    # ------------------------------------------------------------------
    # Move pipeline hook
    # ------------------------------------------------------------------

    def _hook_move_pipeline(self):
        """Intercept every kinematic move via LookAheadQueue.add_move.

        We append a timing_callback to each move. The callback fires
        inside ToolHead._process_moves() after set_junction() has
        populated the velocity profile (start_v, cruise_v, end_v,
        accel_t, cruise_t, decel_t).
        """
        orig_add = self.toolhead.lookahead.add_move
        dcc = self
        def _add_move_with_dcc(move):
            if dcc.enabled and move.is_kinematic_move:
                m = move  # prevent closure over loop var
                move.timing_callbacks.append(
                    lambda pt: dcc._on_move_flush(m, pt))
            orig_add(move)
        self.toolhead.lookahead.add_move = _add_move_with_dcc

    # ------------------------------------------------------------------
    # Velocity zone logic
    # ------------------------------------------------------------------

    def _velocity_to_zone(self, velocity):
        """Map a velocity magnitude to a zone index."""
        for i, threshold in enumerate(self.zone_velocities):
            if velocity < threshold:
                return i
        return len(self.zone_velocities)

    def _find_zone_crossings(self, move, move_start_time):
        """Return [(print_time, zone_index), ...] for all velocity zone
        boundary crossings within a move's accel and decel phases.

        Cruise phase has constant velocity — no crossings possible.
        """
        crossings = []
        phases = []
        t = move_start_time
        if move.accel_t > 0:
            phases.append((t, move.accel_t, move.start_v, move.cruise_v))
            t += move.accel_t
        t += move.cruise_t  # skip cruise
        if move.decel_t > 0:
            phases.append((t, move.decel_t, move.cruise_v, move.end_v))
        for phase_start, duration, v0, v1 in phases:
            if duration <= 0 or v0 == v1:
                continue
            rate = (v1 - v0) / duration
            for i, threshold in enumerate(self.zone_velocities):
                t_cross = (threshold - v0) / rate
                if 0.0 < t_cross < duration:
                    zone = (i + 1) if rate > 0 else i
                    crossings.append((phase_start + t_cross, zone))
        crossings.sort()
        return crossings

    def _is_reversal(self, move):
        """Detect direction reversal by comparing axes_r dot product."""
        if self.prev_axes_r is None:
            return False
        dot = sum(a * b for a, b in
                  zip(move.axes_r[:3], self.prev_axes_r))
        return dot < -0.5

    # ------------------------------------------------------------------
    # SPI register writes
    # ------------------------------------------------------------------

    def _write_registers(self, reg_dict, print_time):
        """Schedule timed SPI writes for a set of TMC registers."""
        t = max(0.0, print_time - self.spi_lead_time)
        for reg_name, val in reg_dict.items():
            self.mcu_tmc.set_register(reg_name, val, t)

    def _apply_zone(self, zone, print_time):
        """Switch to a zone if not already active."""
        if zone == self.current_zone:
            return
        regs = self.zone_registers[zone]
        if regs:
            self._write_registers(regs, print_time)
            self.current_zone = zone
            logging.debug("DCC %s: zone %d at t=%.6f",
                          self.stepper_name, zone, print_time)

    # ------------------------------------------------------------------
    # Per-move callback (fires during _process_moves)
    # ------------------------------------------------------------------

    def _on_move_flush(self, move, next_move_time):
        """Analyze a flushed move's velocity profile and schedule
        TMC register changes at zone boundaries.

        Args:
            move: The Move object (has start_v, cruise_v, end_v,
                  accel_t, cruise_t, decel_t, axes_r, move_d).
            next_move_time: print_time at the END of this move.
        """
        if not self.enabled:
            return
        duration = move.accel_t + move.cruise_t + move.decel_t
        move_start = next_move_time - duration
        is_rev = self._is_reversal(move)
        # --- Reversal handling ---
        if is_rev and (self.reversal_registers or self.reversal_irun_regs):
            rev_time = move_start - self.reversal_lead_time
            if self.reversal_registers:
                self._write_registers(self.reversal_registers, rev_time)
            if self.reversal_irun_regs:
                self._write_registers(self.reversal_irun_regs, rev_time)
            self.in_reversal = True
            self.current_zone = -1
            logging.debug("DCC %s: reversal at t=%.6f",
                          self.stepper_name, move_start)
        if self.in_reversal:
            # End reversal: restore zone profile + normal current
            rev_end = move_start + self.reversal_hold_time
            zone = self._velocity_to_zone(move.start_v)
            regs = self.zone_registers[zone]
            if regs:
                self._write_registers(regs, rev_end)
                self.current_zone = zone
            if self.reversal_irun_regs:
                self._write_registers(
                    {'IHOLD_IRUN': self._normal_ihold_irun}, rev_end)
            self.in_reversal = False
        # --- Zone crossings within the move ---
        crossings = self._find_zone_crossings(move, move_start)
        for cross_time, zone in crossings:
            self._apply_zone(zone, cross_time)
        # --- Ensure correct zone if no crossings ---
        if not crossings and not is_rev:
            self._apply_zone(
                self._velocity_to_zone(move.start_v), move_start)
        # Track direction for next reversal detection
        if move.move_d > 0:
            self.prev_axes_r = list(move.axes_r[:3])

    # ------------------------------------------------------------------
    # G-code commands
    # ------------------------------------------------------------------

    def cmd_ENABLE(self, gcmd):
        self.enabled = True
        self.current_zone = -1
        gcmd.respond_info("DCC enabled for %s" % self.stepper_name)

    def cmd_DISABLE(self, gcmd):
        self.enabled = False
        self.current_zone = -1
        self.in_reversal = False
        gcmd.respond_info("DCC disabled for %s" % self.stepper_name)

    def cmd_STATUS(self, gcmd):
        zones = ', '.join('%.1f' % v for v in self.zone_velocities)
        gcmd.respond_info(
            "DCC %s: %s, zone=%d, reversal=%s, "
            "velocities=[%s] mm/s"
            % (self.stepper_name,
               'enabled' if self.enabled else 'disabled',
               self.current_zone, self.in_reversal, zones))

    def get_status(self, eventtime=None):
        return {
            'enabled': self.enabled,
            'current_zone': self.current_zone,
            'in_reversal': self.in_reversal,
        }


def load_config_prefix(config):
    return DynamicChopper(config)
