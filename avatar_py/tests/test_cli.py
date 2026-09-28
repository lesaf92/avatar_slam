import json

from avatar.cli import main


def test_cli_run_writes_metrics(tmp_path, capsys):
    out = tmp_path / "r.json"
    assert main(["run", "--mode", "independent", "--duration", "30", "--out", str(out)]) == 0
    doc = json.loads(out.read_text())
    assert doc["mode"] == "independent" and "ate_local_m" in doc["metrics"]
    assert "commit" in doc["provenance"]


def test_cli_export_viz(tmp_path):
    out = tmp_path / "v.json"
    assert main(["export-viz", "--duration", "40", "--out", str(out)]) == 0
    doc = json.loads(out.read_text())
    assert doc["structures"] and len(doc["agents"]) == 5
    assert all(a["gt"] for a in doc["agents"])
