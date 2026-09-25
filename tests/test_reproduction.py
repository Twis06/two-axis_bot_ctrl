"""R4: report headers must not crash or claim 'clean' when run outside a git checkout."""
import unittest

from exp import manifest as MF


class TestGitHeaderText(unittest.TestCase):
    NO_GIT = dict(commit=None, dirty=None, dirty_sources=None)

    def test_no_git_is_reported_as_unknown_not_clean(self):
        for g in (self.NO_GIT, None):
            self.assertIn("not a git checkout", MF.git_commit_text(g))
            self.assertNotEqual(MF.git_sources_text(g), "clean")
            self.assertIn("unknown", MF.git_dirty_text(g))

    def test_git_checkout(self):
        g = dict(commit="0123456789abcdef", dirty=True, dirty_sources=["ctrl/baseline.py"])
        self.assertEqual(MF.git_commit_text(g), "0123456789ab")
        self.assertEqual(MF.git_sources_text(g), "MODIFIED: ctrl/baseline.py")
        self.assertEqual(MF.git_dirty_text(g), "ctrl/baseline.py")
        clean = dict(g, dirty=False, dirty_sources=[])
        self.assertEqual(MF.git_sources_text(clean), "clean")
        self.assertEqual(MF.git_dirty_text(clean), "none")


if __name__ == "__main__":
    unittest.main()
