#!/usr/bin/env python3
"""生成“三维仿真回放.html”：10 月 7 日动画里的 CAD 模型 + three.js 脚本包，配上仿真回放界面

用法：python build_viewer.py <输入 html> <输出 html>
  输入可以是：
    a) 10 月 7 日的原动画“完整赛场动画.html”
    b) 已经生成过的“三维仿真回放.html”（只想改界面代码时用这个，不需要原动画）
  做的事：
    - CAD 模型原样保留（网格合并后 gzip 压缩，页面打开时用浏览器自带的解压，文件从 5.3 MB 缩到 1.8 MB）
    - 脚本包里原来的“演示动画逻辑”换成 src/viewer_app.js（它直接用包里的 three.js，名字是压缩过的，见文件开头的别名表）
    - 页面样式和结构换成 src/viewer.css、src/viewer_body.html
    - 数据通过 <script src="replay_latest.js"> 读入（仿真程序每次运行都会重写这个文件）
"""
import base64
import gzip
import json
import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
src_html, out_html = Path(sys.argv[1]), Path(sys.argv[2])
s = src_html.read_text(encoding="utf-8")

APP_BEGIN, APP_END = "/*__VIEWER_APP_BEGIN__*/", "/*__VIEWER_APP_END__*/"
app = (HERE / "src" / "viewer_app.js").read_text(encoding="utf-8")
assert "</script" not in app
app = APP_BEGIN + "\n" + app + "\n" + APP_END
BOOT_HEAD = "window.__startViewer = function () {\n"

if BOOT_HEAD in s:
    # b) 已生成的回放页：取出 CAD 模型和脚本包，只换界面代码
    model_js = re.search(r"<script>(window\.CAD_MODEL=.*?)</script>", s, re.S).group(1)
    a = s.index(BOOT_HEAD) + len(BOOT_HEAD)
    bundle = s[a : s.index("\n};</script>", a)]
    bundle = bundle[: bundle.index(APP_BEGIN)] + app + bundle[bundle.index(APP_END) + len(APP_END) :]
else:
    # a) 原动画
    scripts = [(m.start(), m.end()) for m in re.finditer(r"<script>(.*?)</script>", s, re.S)]
    assert len(scripts) == 2, f"原动画应该正好有 2 段脚本，实际 {len(scripts)}"
    model_js = s[scripts[0][0] + 8 : scripts[0][1] - 9]
    bundle = s[scripts[1][0] + 8 : scripts[1][1] - 9]
    assert model_js.startswith("window.CAD_MODEL="), "第一段脚本不是 CAD 模型"
    start = bundle.index("var Rd=Gr(fd())")          # 原动画逻辑从这里开始，到脚本包末尾的 })(); 为止
    end = bundle.index("})();", start)
    bundle = bundle[:start] + "\n" + app + "\n" + bundle[end:]
    model = json.loads(model_js[len("window.CAD_MODEL=") :].strip().rstrip(";"))
    blob, off = bytearray(), 0
    for m in model["meshes"]:
        raw = base64.b64decode(m.pop("data"))
        assert len(raw) == m["count"] * 12
        m["off"], m["len"] = off, len(raw)
        blob += raw
        off += len(raw)
    gz = base64.b64encode(gzip.compress(bytes(blob), 9, mtime=0)).decode()
    model_js = "window.CAD_MODEL=" + json.dumps(model, ensure_ascii=False, separators=(",", ":")) + ';\nwindow.CAD_MESH_GZ="' + gz + '";'

boot = """(async () => {
  const fail = (msg) => { const el = document.getElementById("loading"); el.classList.remove("done"); el.innerHTML = "<span>" + msg + "</span>"; };
  if (typeof DecompressionStream === "undefined") { fail("这个浏览器太旧，打不开三维模型：请用 Edge 或 Chrome 打开本页面"); return; }
  try {
    const b = atob(window.CAD_MESH_GZ), u = new Uint8Array(b.length);
    for (let i = 0; i < b.length; i++) u[i] = b.charCodeAt(i);
    const buf = await new Response(new Blob([u]).stream().pipeThrough(new DecompressionStream("gzip"))).arrayBuffer();
    for (const m of window.CAD_MODEL.meshes) m.bytes = new Uint8Array(buf, m.off, m.len);
    window.CAD_MESH_GZ = null;
    window.__startViewer();
  } catch (e) { console.error(e); fail("三维页面启动失败：" + e.message); }
})();"""

css = (HERE / "src" / "viewer.css").read_text(encoding="utf-8")
body = (HERE / "src" / "viewer_body.html").read_text(encoding="utf-8")

out = f"""<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>三维仿真回放</title>
<style>
{css}
</style>
</head>
<body>
{body}
<script>{model_js}</script>
<script>window.SIM_REPLAY = window.SIM_REPLAY || null;</script>
<script src="replay_latest.js"></script>
<script>{BOOT_HEAD}{bundle}
}};</script>
<script>{boot}</script>
</body>
</html>
"""
out_html.write_text(out, encoding="utf-8")
print(f"写出 {out_html}  {len(out.encode('utf-8')) / 1e6:.2f} MB")
