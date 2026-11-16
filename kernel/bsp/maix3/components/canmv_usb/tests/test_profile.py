"""Host C parser regression using a real configuration, without a USB device.

Run with python3 test_profile.py. Uses the local C compiler; records no audio.
"""
import ctypes
import os
from pathlib import Path
import random
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
# Anonymized device configuration captured on a Windows host, 08bb:2902.
# UAC1 without IAD, two alternate-setting-1 streams, plus unrelated HID.
CONFIG = bytes.fromhex(
    "0902fd0004010080320904000000010100000a240100016400020102"
    "0c24020101010002030000000c240202010200010100000009240306"
    "010300090009240307010100080007240508010a000a2406090f0101"
    "0202000924060a02014300000924060d02010300000d24040f02010d"
    "02030000000009040100000102000009040101010102000007240101"
    "0101000e2402010202100280bb0044ac0009050109c8000100000725"
    "01010101000904020000010200000904020101010200000724010701"
    "01000e2402010102100280bb0044ac00090582096400010000072501"
    "01000000090403000103000000092100010001223c0007058703040002"
)


class Alt(ctypes.Structure):
    _fields_ = [(n, ctypes.c_uint8) for n in
                ("interface", "setting", "channels", "endpoint", "set_rate")]
    _fields_.append(("max_packet", ctypes.c_uint16))


class Profile(ctypes.Structure):
    _fields_ = [("capture", Alt), ("playback", Alt)]


class ProfileTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory()
        cls.addClassCleanup(cls.temp.cleanup)
        lib = Path(cls.temp.name, "profile.so")
        subprocess.run([os.environ.get("CC", "cc"), "-std=c11", "-Wall", "-Wextra",
                        "-Werror", "-fPIC", "-shared", str(ROOT / "uac1_profile.c"),
                        "-o", str(lib)], check=True)
        cls.parse = ctypes.CDLL(str(lib)).uac1_parse
        cls.parse.argtypes = [ctypes.c_char_p, ctypes.c_size_t, ctypes.c_uint8,
                              ctypes.POINTER(Profile)]
        cls.parse.restype = ctypes.c_int

    def profile(self, raw, ctrl=0):
        out = Profile()
        rc = self.parse(bytes(raw), len(raw), ctrl, ctypes.byref(out))
        return rc, out

    def test_real_48k_capture_and_playback(self):
        rc, p = self.profile(CONFIG)
        self.assertEqual(rc, 0)
        self.assertEqual((p.capture.interface, p.capture.setting, p.capture.endpoint,
                          p.capture.channels, p.capture.max_packet, p.capture.set_rate),
                         (2, 1, 0x82, 1, 100, 1))
        self.assertEqual((p.playback.interface, p.playback.setting, p.playback.endpoint,
                          p.playback.channels, p.playback.max_packet, p.playback.set_rate),
                         (1, 1, 0x01, 2, 200, 1))

    def test_nonfirst_discrete_rate(self):
        raw = CONFIG.replace(bytes.fromhex("0280bb0044ac00"),
                             bytes.fromhex("0244ac0080bb00"))
        rc, p = self.profile(raw)
        self.assertEqual((rc, p.capture.channels, p.playback.channels), (0, 1, 2))

    def test_continuous_rate_range(self):
        lo = bytes.fromhex("0280bb0044ac00")
        for hi, ok in (("80bb00", True), ("44ac00", False)):
            rc, _ = self.profile(CONFIG.replace(lo, bytes.fromhex("00803e00" + hi)))
            self.assertEqual(rc == 0, ok)

    def test_multirate_alternate_requires_an_endpoint_frequency_control(self):
        raw = CONFIG.replace(bytes.fromhex("07250101010100"),
                             bytes.fromhex("07250100010100"))
        rc, p = self.profile(raw)
        self.assertEqual((rc, p.capture.channels, p.playback.channels), (0, 1, 0))

    def test_other_audio_function_and_composite_interface(self):
        self.assertNotEqual(self.profile(CONFIG, 3)[0], 0)
        # AC owns only interface 2; unrelated output must not appear.
        raw = CONFIG.replace(bytes.fromhex("6400020102"), bytes.fromhex("6400020205"))
        rc, p = self.profile(raw)
        self.assertEqual((rc, p.capture.channels, p.playback.channels), (0, 1, 0))

    def test_reject_unsupported_playback_keep_mic(self):
        signature = bytes.fromhex("09050109c800010000")
        cases = ["09050105c800010000",  # asynchronous OUT requires feedback
                 "09050109c800010082",  # provided synch address
                 "09050119c800010000",  # feedback endpoint usage
                 "09050109c800020000",  # not 1-ms Full Speed interval
                 "090501090400010000",  # max packet too small for selected format
                 "09050009c800010000",  # endpoint zero
                 "09050109c808010000"]  # additional HS transactions
        for value in cases:
            with self.subTest(value=value):
                rc, p = self.profile(CONFIG.replace(signature, bytes.fromhex(value)))
                self.assertEqual((rc, p.capture.channels, p.playback.channels), (0, 1, 0))

    def test_reject_uac2_unqualified_pcm_rates_and_truncations(self):
        original = bytes.fromhex("0a24010001")
        self.assertNotEqual(self.profile(CONFIG.replace(original, bytes.fromhex("0a24010002")))[0], 0)
        for rate in ("02803e0044ac00", "ff80bb0044ac00"):
            self.assertNotEqual(self.profile(CONFIG.replace(bytes.fromhex("0280bb0044ac00"),
                                                           bytes.fromhex(rate)))[0], 0)
        for length in range(len(CONFIG)):
            self.assertNotEqual(self.profile(CONFIG[:length])[0], 0)
        for i in range(9, len(CONFIG)):
            if CONFIG[i:i+2] == b"\x09\x04":
                raw = bytearray(CONFIG)
                raw[i] = 1
                self.assertNotEqual(self.profile(raw)[0], 0)

    def test_malformed_inputs_never_expose_an_out_of_range_profile(self):
        rand = random.Random(230)
        for _ in range(5000):
            raw = bytearray(CONFIG)
            for _ in range(4):
                raw[rand.randrange(len(raw))] = rand.randrange(256)
            rc, p = self.profile(raw)
            if rc == 0:
                for alt in (p.capture, p.playback):
                    if alt.channels:
                        self.assertIn(alt.channels, (1, 2))
                        self.assertLessEqual(alt.max_packet, 256)
                        self.assertGreaterEqual(alt.max_packet, alt.channels * 96)


if __name__ == "__main__":
    unittest.main()
