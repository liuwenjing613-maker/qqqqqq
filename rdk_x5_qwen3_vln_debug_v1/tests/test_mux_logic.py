#!/usr/bin/env python3
from __future__ import annotations

import unittest

from intervention.mux_logic import choose_source


BASE = dict(
    mode_age_sec=0.1,
    ego_age_sec=0.1,
    map_age_sec=0.1,
    safety_age_sec=0.1,
    emergency_reverse_active=False,
    mode_timeout_sec=2.0,
    ego_timeout_sec=0.45,
    map_timeout_sec=0.45,
    safety_status_timeout_sec=0.8,
    safety_override_enabled=True,
    require_fresh_safety_status_in_map=True,
)


class MuxLogicTests(unittest.TestCase):
    def pick(self, mode, **kw):
        values = dict(BASE)
        values.update(kw)
        return choose_source(requested_mode=mode, **values)

    def test_ego_selects_only_fresh_ego(self):
        self.assertEqual(self.pick("EGO").source, "EGO")
        self.assertEqual(
            self.pick("EGO", ego_age_sec=0.8).source,
            "ZERO",
        )

    def test_map_selects_only_fresh_map(self):
        self.assertEqual(self.pick("MAP").source, "MAP")
        self.assertEqual(
            self.pick("MAP", map_age_sec=0.8).reason,
            "map_cmd_stale",
        )

    def test_map_fails_closed_when_safety_status_stale(self):
        d = self.pick("MAP", safety_age_sec=1.0)
        self.assertEqual(d.source, "ZERO")
        self.assertEqual(d.reason, "map_safety_status_stale")

    def test_emergency_reverse_overrides_map_with_ego(self):
        d = self.pick("MAP", emergency_reverse_active=True)
        self.assertEqual(d.source, "EGO")
        self.assertEqual(d.effective_mode, "EGO_SAFETY")

    def test_emergency_reverse_with_stale_ego_holds(self):
        d = self.pick(
            "MAP", emergency_reverse_active=True, ego_age_sec=0.9
        )
        self.assertEqual(d.source, "ZERO")
        self.assertEqual(d.reason, "emergency_reverse_ego_stale")

    def test_stale_manager_mode_holds(self):
        d = self.pick("MAP", mode_age_sec=3.0)
        self.assertEqual(d.source, "ZERO")
        self.assertEqual(d.reason, "mode_stale")


if __name__ == "__main__":
    unittest.main()
