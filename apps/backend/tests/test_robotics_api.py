"""API integration tests exercising browser URDF import, poses, and ZIP export."""
from __future__ import annotations

import io
import math
import zipfile
from pathlib import Path

import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient

from opencad_server.robotics_router import router
from opencad.robotics import (
    RobotBuilder, RobotGeometry, RobotVisual, export_urdf,
)


@pytest.fixture()
def client() -> TestClient:
    """Create an isolated robotics router, without OCCT or agent services."""
    app = FastAPI()
    app.include_router(router, prefix="/robotics")
    return TestClient(app)


def _fixture() -> str:
    """Provide a two-link URDF with nontrivial rotations and a visual STL."""
    return """<robot name="sample">
      <link name="base"><visual><geometry><box size="0.02 0.03 0.04"/></geometry></visual></link>
      <link name="tool">
        <visual><origin xyz="0.01 0 0"/><geometry><mesh filename="package://assets/tool.stl"/></geometry></visual>
        <collision><geometry><sphere radius="0.01"/></geometry></collision>
      </link>
      <joint name="hinge" type="revolute">
        <parent link="base"/><child link="tool"/>
        <origin xyz="0.1 0 0" rpy="0 0 1.5707963267948966"/>
        <axis xyz="0 0 1"/>
        <limit lower="-1" upper="1" effort="2" velocity="3"/>
      </joint>
    </robot>"""


def _bundle() -> bytes:
    """Bundle a robot and one ASCII STL triangle under its nested asset root."""
    output = io.BytesIO()
    with zipfile.ZipFile(output, "w", zipfile.ZIP_DEFLATED) as z:
        z.writestr("robot/robot.urdf", _fixture())
        z.writestr("robot/assets/tool.stl", """solid triangle
 facet normal 0 0 1
  outer loop
   vertex 0 0 0
   vertex 0.01 0 0
   vertex 0 0.01 0
  endloop
 endfacet
endsolid triangle
""")
    return output.getvalue()


def test_browser_robot_roundtrip(client: TestClient) -> None:
    """ZIP import → pose articulation → export → reimport retains geometry."""
    original = _bundle()
    imported = client.post(
        "/robotics/import", params={"filename": "sample.zip"},
        content=original, headers={"Content-Type": "application/octet-stream"},
    )
    assert imported.status_code == 200, imported.text
    data = imported.json()
    assert data["export_enabled"]
    assert data["motion_enabled"]
    assert len(data["meshes"]) == 1
    assert data["meshes"][0]["shapeId"] == "urdf:tool:visual:0"
    # 0.01 source vertices (meters) become 10 mm.
    assert data["meshes"][0]["vertices"][3:6] == pytest.approx([10, 0, 0])
    assert any("inertial" in warning for warning in data["warnings"])
    base = client.post("/robotics/poses", json={"robot": data["robot"], "positions": {}})
    moved = client.post("/robotics/poses", json={"robot": data["robot"], "positions": {"hinge": 0.5}})
    assert base.status_code == moved.status_code == 200
    before = base.json()["shape_transforms"]["urdf:tool:visual:0"]
    after = moved.json()["shape_transforms"]["urdf:tool:visual:0"]
    assert before["translation_mm"] != after["translation_mm"]
    exported = client.post("/robotics/export", params={"filename": "sample.zip"},
                           content=original)
    assert exported.status_code == 200, exported.text
    assert exported.headers["content-type"] == "application/zip"
    with zipfile.ZipFile(io.BytesIO(exported.content)) as z:
        files = set(z.namelist())
        assert "robot.urdf" in files
        assert "robot.opencad.json" in files
        assert len([f for f in files if f.startswith("meshes/")]) == 1
        rerender = client.post("/robotics/import", params={"filename": "rebuilt.zip"},
                               content=exported.content)
        assert rerender.status_code == 200, rerender.text
        assert len(rerender.json()["robot"]["joints"]) == 1


def test_single_file_robot_and_invalid_export(client: TestClient) -> None:
    """Plain primitive URDF loads and incomplete moving limits block export."""
    xml = """<robot name="plain">
      <link name="a"><visual><geometry><box size="0.1 0.1 0.1"/></geometry></visual></link>
      <link name="b"/>
      <joint name="slide" type="prismatic">
        <parent link="a"/><child link="b"/><limit lower="0" upper="0.1"/>
      </joint>
    </robot>"""
    result = client.post("/robotics/import", params={"filename": "plain.urdf"},
                         content=xml.encode())
    assert result.status_code == 200, result.text
    assert result.json()["motion_enabled"]
    assert not result.json()["export_enabled"]
    assert "mandatory URDF limit" in result.json()["export_blocker"]


def test_reject_unsafe_zip_entries(client: TestClient) -> None:
    """Uploaded archives cannot escape the sandbox or contain symbolic links."""
    output = io.BytesIO()
    with zipfile.ZipFile(output, "w") as z:
        z.writestr("robot.urdf", "<robot name='test'><link name='base'/></robot>")
        z.writestr("../escape.stl", "anything")
    result = client.post("/robotics/import", params={"filename": "bad.zip"},
                         content=output.getvalue())
    assert result.status_code == 422
    assert "Unsafe ZIP" in result.text


def test_continuous_and_unresolved_joint_gate(client: TestClient) -> None:
    """Continuous motion is unlimited; unresolved mimic or loops disable motion."""
    xml = """<robot name="spinner">
      <link name="base"/><link name="wheel"/>
      <joint name="spin" type="continuous"><parent link="base"/><child link="wheel"/>
        <axis xyz="1 0 0"/><limit effort="1" velocity="2"/>
      </joint>
    </robot>"""
    result = client.post("/robotics/import", params={"filename": "spinner.urdf"}, content=xml)
    assert result.status_code == 200
    assert result.json()["motion_enabled"]
    response = client.post("/robotics/poses",
                           json={"robot": result.json()["robot"], "positions": {"spin": 35.0}})
    assert response.status_code == 200
    xml = xml.replace('<axis xyz="1 0 0"/>', '<axis xyz="1 0 0"/><mimic joint="spin"/>')
    bad = client.post("/robotics/import", params={"filename": "bad.urdf"}, content=xml)
    assert bad.status_code == 200
    assert not bad.json()["motion_enabled"]


def test_reject_oversized_upload(client: TestClient) -> None:
    """Large file bodies are rejected before being written to disk."""
    result = client.post("/robotics/import", params={"filename": "huge.urdf"},
                         content=b"x" * (24 * 1024 * 1024 + 1))
    assert result.status_code == 413
