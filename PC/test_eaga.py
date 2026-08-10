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


if __name__ == "__main__":
    unittest.main()
