import struct
import unittest

import numpy as np

from PC import sae_pc_hil as sae


class EagaStructureTests(unittest.TestCase):
    def setUp(self):
        self.original_fitness = sae.fitness
        self.original_incumbent = sae.incumbent_chrom.copy()
        self.random_state = np.random.get_state()

    def tearDown(self):
        sae.fitness = self.original_fitness
        sae.incumbent_chrom = self.original_incumbent
        np.random.set_state(self.random_state)

    def test_rank_diffusion_increases_with_worse_rank(self):
        sigma = [sae.rank_diffusion_sigma(rank) for rank in range(sae.EA_POP)]
        self.assertTrue(all(left < right for left, right in zip(sigma, sigma[1:])))

    def test_online_eaga_uses_subpopulations_and_bounded_archive(self):
        target = sae.CHROM_INCUMBENT + np.array([0.08, -0.03, 0.02, 0.04, -0.05])

        def cheap_fitness(chromosome):
            score = float(np.sum((chromosome - target) ** 2))
            return score, chromosome[:4].copy()

        sae.fitness = cheap_fitness
        sae.incumbent_chrom = sae.CHROM_INCUMBENT.copy()
        np.random.seed(7)

        elite, gain, score, _, diagnostics = sae.eaga_online(return_diagnostics=True)

        self.assertEqual(diagnostics["subpopulation_count"], 2)
        self.assertEqual(diagnostics["population_shape"], (2, 10, 5))
        self.assertGreater(diagnostics["archive_size"], 0)
        self.assertLessEqual(diagnostics["archive_size"], sae.EA_ARCHIVE_SIZE)
        self.assertEqual(diagnostics["archive_scores"],
                         sorted(diagnostics["archive_scores"]))
        self.assertGreater(diagnostics["sigma_worst"], diagnostics["sigma_best"])
        self.assertTrue(np.all(np.isfinite(elite)))
        self.assertTrue(np.all(np.isfinite(gain)))
        self.assertTrue(np.isfinite(score))


class ScoredControllerProtocolTests(unittest.TestCase):
    def test_extended_frame_round_trip(self):
        gain, riccati = sae.riccati_from_chrom(sae.CHROM_INCUMBENT)
        score = sae.controller_fitness(gain, sae.TAU_FF_INC)
        frame = sae.frame_gain_downlink(gain, sae.TAU_FF_INC, score, riccati)

        decoded_gain, decoded_ff, decoded_score, decoded_riccati = (
            sae.validate_gain_downlink_frame(frame))

        self.assertEqual(len(frame), 68)
        self.assertEqual(frame[:3], bytes([0xAA, 0x55, 0xA6]))
        np.testing.assert_allclose(decoded_gain, gain, rtol=1e-6)
        self.assertAlmostEqual(decoded_ff, sae.TAU_FF_INC, places=6)
        self.assertAlmostEqual(decoded_score, score, places=6)
        np.testing.assert_allclose(decoded_riccati, riccati, rtol=1e-6)

    def test_status_frame_is_parsed_separately_from_telemetry(self):
        body = bytes([0xAA, 0x55, 0x5D]) + struct.pack(
            "<BBHfI", 1, 0, 7, 0.125, 2)
        frame = body + bytes([sum(body) & 0xFF])
        status, rest, kind = sae.parse_stream(bytearray(frame))
        self.assertEqual(kind, "status")
        self.assertEqual(rest, bytearray())
        self.assertEqual(status[:3], (1, 0, 7))
        self.assertAlmostEqual(status[3], 0.125, places=6)
        self.assertEqual(status[4], 2)


if __name__ == "__main__":
    unittest.main()
