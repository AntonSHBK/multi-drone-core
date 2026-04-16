from pathlib import Path

from setuptools import find_namespace_packages, setup

ROOT = Path(__file__).parent
README = ROOT / "README.md"
REQUIREMENTS = ROOT / "requirements.txt"

long_description = README.read_text(encoding="utf-8") if README.exists() else ""

install_requires = []
if REQUIREMENTS.exists():
    install_requires = [
        line.strip()
        for line in REQUIREMENTS.read_text(encoding="utf-8").splitlines()
        if line.strip() and not line.strip().startswith("#")
    ]

setup(
    name="multi-drone-core",
    version="0.1.0",
    description="Core library for multi-drone control and command execution",
    long_description=long_description,
    long_description_content_type="text/markdown",
    author="multi-drone-core contributors",
    python_requires=">=3.10",
    package_dir={"": "src"},
    packages=find_namespace_packages(where="src", include=["multi_drone_core*"]),
    include_package_data=True,
    install_requires=install_requires,
    license="GPL-3.0-or-later",
)
