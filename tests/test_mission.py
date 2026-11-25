import unittest

from car26.mission import Mission


SETTINGS = {"entry": "full", "object_enabled": True, "shot_timeout_s": 30,
            "collect_timeout_s": 8, "sensor_timeout_s": 5, "minimum_frames": 2,
            "minimum_winner_share": .7}


class MissionTests(unittest.TestCase):
    def make_mission(self):
        return Mission(SETTINGS, lambda start: start == 4)

    def test_wait_start_indefinitely(self):
        mission = self.make_mission()
        self.assertEqual(mission.tick(100000), [])
        self.assertEqual(mission.state, "WAIT_START")

    def test_uncalibrated_start_gets_no_ack(self):
        mission = self.make_mission()
        self.assertEqual(mission.command("24", 0), [])
        self.assertEqual(mission.state, "FAILED")

    def test_skip_goes_to_qr_without_map(self):
        mission = self.make_mission()
        self.assertEqual(mission.command("skip", 0), [("camera", "qr")])

    def test_three_shots_and_map_order(self):
        mission = self.make_mission()
        self.assertEqual(mission.command("4", 0), [("send", b"[ack]")])
        for index, angle in enumerate((0, 45, 90)):
            now = index * 10 + 1
            self.assertEqual(mission.angle, angle)
            self.assertEqual(mission.command("shot", now), [("prescan", None)])
            self.assertEqual(mission.command("shot", now + .01), [])
            self.assertEqual(mission.observe(1, now + .1, {13: .9}, now + .1), [])
            effects = mission.observe(2, now + .2, {13: .9}, now + .2)
            self.assertEqual(effects[0], ("send", b"[ack]"))
        self.assertEqual(effects[1:], [("send", b"[1 13 0C]"), ("camera", "qr")])
        self.assertEqual(mission.state, "QR")

    def test_stale_and_duplicate_frames_do_not_vote(self):
        mission = self.make_mission()
        mission.command("4", 0)
        mission.command("shot", 1)
        mission.observe(1, .9, {13: 1}, 1.1)
        mission.observe(2, 1.1, {13: 1}, 1.2)
        mission.observe(2, 1.1, {13: 1}, 1.3)
        self.assertEqual(mission.votes[13], 1)
        self.assertEqual(mission.tick(7), [])
        self.assertEqual(mission.error, "PRESCAN_SENSOR_STALE")

    def test_empty_frames_are_live_but_not_positive(self):
        mission = self.make_mission()
        mission.command("4", 0)
        for index in range(3):
            start = index * 10 + 1
            mission.command("shot", start)
            for offset in (1, 4, 7):
                mission.observe(offset, start + offset, {}, start + offset)
            effects = mission.tick(start + 8)
            self.assertEqual(effects, [("send", b"[ack]")])
        self.assertEqual(mission.error, "OBSTACLE_VOTE_AMBIGUOUS")

    def test_shot_timeout_no_false_ack(self):
        mission = self.make_mission()
        mission.command("4", 0)
        self.assertEqual(mission.tick(31), [])
        self.assertEqual(mission.error, "SHOT_COMMAND_TIMEOUT")

    def test_late_command_cannot_bypass_deadline(self):
        mission = self.make_mission()
        mission.command("4", 0)
        self.assertEqual(mission.command("shot", 31), [])
        self.assertEqual(mission.error, "SHOT_COMMAND_TIMEOUT")

    def test_late_frame_does_not_extend_collect(self):
        mission = self.make_mission()
        mission.command("4", 0)
        mission.command("shot", 1)
        mission.observe(1, 7, {}, 7)
        self.assertEqual(mission.observe(2, 9.1, {13: 1}, 9.1), [("send", b"[ack]")])
        self.assertFalse(mission.votes)

    def test_qr_four_frames_then_flush_then_handoff(self):
        mission = self.make_mission()
        mission.command("skip", 0)
        effects = mission.qr("payload")
        self.assertEqual(len([a for a, _ in effects if a == "send"]), 4)
        self.assertEqual(effects[-2:], [("flush", None), ("qr_sent", None)])
        self.assertEqual(mission.state, "QR_SENDING")
        self.assertEqual(mission.qr("payload"), [])
        self.assertEqual(mission.qr_sent(), [("camera", "object")])

    def test_ambiguous_winner_is_rejected(self):
        mission = self.make_mission()
        mission.command("4", 0)
        for index in range(3):
            now = index * 10 + 1
            mission.command("shot", now)
            mission.observe(1, now + .1, {1: .8, 3: .8}, now + .1)
            mission.observe(2, now + .2, {1: .8, 3: .8}, now + .2)
        self.assertEqual(mission.error, "OBSTACLE_VOTE_AMBIGUOUS")
