"""Command-line entry for offline dynamic point filtering; no GUI or robot connection."""
import argparse
import json
from pathlib import Path
import sys
ROOT = Path(__file__).resolve().parents[1]


def latest_source():
    pointer = ROOT/'runtime/latest_map.json'
    if pointer.exists():
        return Path(json.loads(pointer.read_text())['folder'])
    return ROOT/'maps/imported_s100_20261027_142818_INVALID'


def main():
    parser = argparse.ArgumentParser(description='离线动态点云滤除 CLI（自研 V2；不连接机器人，不打开 GUI）')
    subs = parser.add_subparsers(dest='action')
    subs.add_parser('list', help='列出本机已保存地图及滤除结果')
    info = subs.add_parser('info', help='查看地图/滤除结果摘要；默认最新滤除结果')
    info.add_argument('source', nargs='?', type=Path)
    run = subs.add_parser('run', help='执行离线 V2 滤除，另存结果，不覆盖原图')
    run.add_argument('source', nargs='?', type=Path, help='原始地图目录；省略时采用正式地图或旧导入地图')
    run.add_argument('--output', type=Path, help='新输出目录（必须不存在）')
    args = parser.parse_args()
    if args.action is None:
        parser.print_help()
        return
    if args.action == 'list':
        for folder in sorted((ROOT/'maps').iterdir()):
            path = folder/'map.json'
            if not path.is_file():
                continue
            try:
                meta = json.loads(path.read_text())
                kind = '滤除结果' if isinstance(meta.get('dynamic_removal'), dict) else '原始地图'
                invalid = meta.get('invalid_reason') or '无'
                print(f'{folder}\n  {kind} | 体素 {meta.get("cells", "?")} | 失效原因 {invalid}')
            except (OSError, ValueError) as exc:
                print(f'{folder}: 无法读取 {exc}', file=sys.stderr)
        return
    if args.action == 'info':
        source = args.source
        if source is None:
            pointer = ROOT/'runtime/latest_dynamic_map.json'
            source = Path(json.loads(pointer.read_text())['folder']) if pointer.exists() else latest_source()
        meta = json.loads((source/'map.json').read_text())
        print(json.dumps(dict(folder=str(source.resolve()), cells=meta.get('cells'),
                              invalid_reason=meta.get('invalid_reason'),
                              validation_only=meta.get('validation_only'),
                              dynamic_removal=meta.get('dynamic_removal')), ensure_ascii=False, indent=2))
        return
    source = (args.source or latest_source()).expanduser().resolve()
    if not (source/'map.json').is_file():
        parser.error(f'地图目录不存在或缺少 map.json: {source}')
    meta = json.loads((source/'map.json').read_text())
    if isinstance(meta.get('dynamic_removal'), dict):
        parser.error('请使用原始地图目录，避免对滤除结果重复处理')
    if args.output is not None and args.output.exists():
        parser.error('输出目录已存在，请指定新目录')
    print(f'算法：自研 V2 多帧射线一致性 + 视角/结构保护\n输入：{source}\n离线处理；原始地图保留。', flush=True)
    from dynamic_map import run
    destination, report = run(source, args.output)
    print(f'结果目录：{destination}\n报告：{destination / "dynamic_report.json"}', flush=True)


if __name__ == '__main__':
    try:
        main()
    except (OSError, ValueError, KeyError) as exc:
        print(f'错误：{exc}', file=sys.stderr)
        sys.exit(1)
