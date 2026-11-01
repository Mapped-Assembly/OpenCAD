"""Invariants for the independently specified hydroponic rack example."""
from __future__ import annotations

import importlib.util
import sys
from pathlib import Path
from unittest.mock import MagicMock, patch

import pytest

SOURCE = Path(__file__).resolve().parents[1] / "examples" / "elfsys_hydroponic_rack.py"
spec = importlib.util.spec_from_file_location("elfsys_hydroponic_rack", SOURCE)
assert spec is not None and spec.loader is not None
model = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = model
spec.loader.exec_module(model)


@pytest.mark.parametrize("levels,sites,parts_expected", [(2, 28, 29), (4, 56, 45)])
def test_configuration(levels: int, sites: int, parts_expected: int) -> None:
    """Check site counts, stable part identities and tray/lid pairs."""
    params = model.HydroponicParameters(levels=levels)
    assert params.total_sites == sites
    with patch.object(model, "Part") as factory:
        factory.return_value = MagicMock()
        components = model.build(params)
    assert len(components) == parts_expected
    assert len({item.name for item in components}) == parts_expected
    assert sum(item.kind == "plant_site_lid" for item in components) == 2 * levels
    assert sum(item.kind == "grow_tray" for item in components) == 2 * levels


def test_invalid_spacing() -> None:
    """Refuse hole patterns that cannot fit inside a lid."""
    with pytest.raises(ValueError, match="Seven plant holes"):
        model.HydroponicParameters(plant_pitch_mm=190).validate()


def test_unsupported_level_count() -> None:
    """Permit only the configurations evidenced by the reference."""
    with pytest.raises(ValueError, match="2 or 4"):
        model.HydroponicParameters(levels=3).validate()


def test_lid_has_seven_through_cuts() -> None:
    """Verify that holes are modeled as boolean operations."""
    with patch.object(model, "Part") as factory:
        mock = MagicMock()
        mock.box.return_value = mock
        mock.cylinder.return_value = mock
        mock.translate.return_value = mock
        factory.return_value = mock
        model.perforated_lid(model.HydroponicParameters(), "lid", 0, 0, 800)
    assert mock.cut.call_count == 7
