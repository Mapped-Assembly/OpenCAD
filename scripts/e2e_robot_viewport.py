"""Run a real Chromium URDF viewport import → pose → export → re-import test.

Intended for CI with the OpenCAD FastAPI backend and Vite frontend running.
Install the optional browser dependency with uv --with playwright.
"""
from __future__ import annotations

import io
import os
import tempfile
import zipfile
from pathlib import Path

from playwright.sync_api import Page, Response, sync_playwright


def create_robot_fixture() -> bytes:
    """Create a compact nested ROS-style URDF bundle and STL triangle."""
    urdf = """<robot name="browser_bot">
      <link name="base">
        <visual><geometry><box size="0.02 0.02 0.02"/></geometry></visual>
      </link>
      <link name="tip">
        <visual><origin xyz="0.01 0 0"/><geometry>
          <mesh filename="package://assets/tip.stl"/>
        </geometry></visual>
      </link>
      <joint name="hinge" type="revolute">
        <parent link="base"/><child link="tip"/>
        <origin xyz="0.1 0 0"/><axis xyz="0 0 1"/>
        <limit lower="-1" upper="1" effort="2" velocity="3"/>
      </joint>
    </robot>"""
    stl = """solid triangle
 facet normal 0 0 1
  outer loop
   vertex 0 0 0
   vertex 0.01 0 0
   vertex 0 0.01 0
  endloop
 endfacet
endsolid triangle
"""
    out = io.BytesIO()
    with zipfile.ZipFile(out, mode="w", compression=zipfile.ZIP_DEFLATED) as archive:
        archive.writestr("robot/robot.urdf", urdf)
        archive.writestr("robot/assets/tip.stl", stl)
    return out.getvalue()


def shape_translation(response: Response) -> list[float]:
    """Return the world-space visual translation from an API FK response."""
    payload = response.json()
    return payload["shape_transforms"]["urdf:tip:visual:0"]["translation_mm"]


def run_browser_test(page: Page, url: str, output: Path) -> None:
    """Exercise the real React robot workspace and ZIP export in Chromium."""
    page.goto(url, wait_until="networkidle")
    page.get_by_role("button", name="URDF Robot Workspace").click()
    page.get_by_role("heading", name="OpenCAD Robotics").wait_for()

    file_input = page.get_by_label("Import URDF robot")
    with page.expect_response(lambda resp: "/robotics/poses" in resp.url and resp.status == 200) as initial_event:
        file_input.set_input_files({
            "name": "browser_bot.zip",
            "mimeType": "application/zip",
            "buffer": create_robot_fixture(),
        })
    initial = shape_translation(initial_event.value)
    page.get_by_text("browser_bot").first.wait_for()
    page.get_by_role("treeitem", name="tip").click()
    page.get_by_text("Selected link").wait_for()
    assert page.get_by_text("Visuals: 1").count() > 0
    joint = page.get_by_label("Joint hinge")
    with page.expect_response(lambda resp: "/robotics/poses" in resp.url and resp.status == 200) as moved_event:
        joint.fill("0.5")
    moved = shape_translation(moved_event.value)
    assert initial != moved, f"Joint change did not affect visual pose: {initial} vs {moved}"
    with page.expect_response(lambda resp: "/robotics/poses" in resp.url and resp.status == 200):
        page.get_by_role("button", name="Reset joints").click()
    assert float(joint.input_value()) == 0.0

    with page.expect_download() as download_event:
        page.get_by_role("button", name="Export URDF bundle").click()
    download = download_event.value
    downloaded = output / "browser_robot_roundtrip.zip"
    download.save_as(downloaded)
    with zipfile.ZipFile(downloaded) as archive:
        names = archive.namelist()
        assert "robot.urdf" in names, names
        assert "robot.opencad.json" in names, names
        assert any(name.startswith("meshes/") for name in names), names

    # Re-import exactly what the user downloaded; also proves mesh paths work.
    with page.expect_response(lambda resp: "/robotics/import" in resp.url) as reimport_event:
        file_input.set_input_files(str(downloaded))
    assert reimport_event.value.status == 200, reimport_event.value.text()
    page.get_by_role("treeitem", name="tip").wait_for()
    assert page.get_by_role("button", name="Export URDF bundle").is_enabled()
    assert page.get_by_role("alert").count() == 0, "Unexpected error after export/re-import"
    page.screenshot(path=str(output / "robot_workflow.png"), full_page=True)


def main() -> None:
    """Launch headless Chromium against the locally running OpenCAD services."""
    from playwright.sync_api import sync_playwright

    base_url = os.environ.get("OPENCAD_BROWSER_BASE_URL", "http://127.0.0.1:5173")
    with tempfile.TemporaryDirectory(prefix="opencad-browser-") as tmp:
        screenshots = Path(os.environ.get("OPENCAD_BROWSER_EVIDENCE", tmp))
        screenshots.mkdir(parents=True, exist_ok=True)
        with sync_playwright() as playwright:
            browser = playwright.chromium.launch(headless=True)
            page = browser.new_page(accept_downloads=True)
            try:
                run_browser_test(page, base_url, screenshots)
            finally:
                browser.close()
        print("URDF browser E2E passed: upload -> articulate -> reset -> export -> reimport")
        print(f"Evidence directory: {screenshots}")


if __name__ == "__main__":
    main()
