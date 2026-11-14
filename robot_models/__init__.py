"""Model locations resolve relative to this project, independent of working directory."""
from pathlib import Path

ARM_XML = Path(__file__).resolve().parent / 'mr_jos_6dof.xml'
