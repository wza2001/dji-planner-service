import pytest
import io
import zipfile

# Skip the test since we don't have the hardcoded Windows test files in this CI/CD container
@pytest.mark.skip(reason="Needs local windows filesystem paths which are not present in CI")
def test_generate_kmz_from_file_pipeline():
    pass
