"""Shared pytest setup: put the repo root on sys.path so tests can
import ``xDesigner`` without an editable install."""
import os
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)
