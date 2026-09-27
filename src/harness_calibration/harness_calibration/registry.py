"""Module registry: which AprilTag id belongs to which module (from descriptors/*.yaml)."""
import glob
import os

import yaml

DESCRIPTOR_DIR = os.path.expanduser("~/bunny-harness-dev/descriptors")


def load(descriptor_dir=DESCRIPTOR_DIR):
    """Return {tag_id: {"module": name, "role": role, "size_mm": s, "descriptor": doc}}."""
    reg = {}
    for fn in sorted(glob.glob(os.path.join(descriptor_dir, "*.yaml"))):
        with open(fn) as f:
            doc = yaml.safe_load(f) or {}
        for e in (doc.get("tags") or {}).get("layout") or []:
            if e.get("id") is None:
                continue                      # id not known yet (read at first sight by 'locate')
            reg[int(e["id"])] = {"module": doc.get("module", os.path.basename(fn)[:-5]), "role": e.get("role", "main"),
                                 "size_mm": e.get("size_mm"), "descriptor": doc, "file": fn}
    return reg


def descriptor(module, descriptor_dir=DESCRIPTOR_DIR):
    fn = os.path.join(descriptor_dir, f"{module}.yaml")
    with open(fn) as f:
        return yaml.safe_load(f), fn


def set_tag_id(module, role, tag_id, descriptor_dir=DESCRIPTOR_DIR):
    """Write a discovered tag id into the descriptor (text edit keeps comments)."""
    fn = os.path.join(descriptor_dir, f"{module}.yaml")
    with open(fn) as f:
        txt = f.read()
    new = txt.replace(f"{{id: null, role: {role},", f"{{id: {int(tag_id)}, role: {role},", 1)
    if new != txt:
        with open(fn, "w") as f:
            f.write(new)
    return new != txt
