# 把 replay_template.html 转成 C 头文件 replay_template.h（改了 HTML 模板后运行一次：python gen_template.py）
import json, pathlib
here = pathlib.Path(__file__).parent
html = (here / 'replay_template.html').read_text(encoding='utf-8')
head, tail = html.split('/*__DATA__*/')
def c_str(name, text):
    out = [f'static const char {name}[] =']
    for line in text.splitlines(keepends=True):
        out.append('    ' + json.dumps(line, ensure_ascii=False))
    return '\n'.join(out) + ';\n'
(here / 'replay_template.h').write_text('/* 由 gen_template.py 从 replay_template.html 自动生成，不要手改 */\n' + c_str('REPLAY_HEAD', head) + c_str('REPLAY_TAIL', tail), encoding='utf-8')
print('ok')
