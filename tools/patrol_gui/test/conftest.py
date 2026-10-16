"""让 patrol_gui 包可在测试里 import（tools/ 加入 sys.path）。"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.abspath(os.path.join(HERE, "..", ".."))   # tools/
SRC_PATROL = os.path.abspath(os.path.join(HERE, "..", "..", "..",
                                          "src", "go2_patrol_patrol"))
for path in (TOOLS, SRC_PATROL):
    if path not in sys.path:
        sys.path.insert(0, path)
