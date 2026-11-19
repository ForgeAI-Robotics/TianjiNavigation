"""Mapping dispatcher: FAST-LIO2 by default, explicit legacy rollback."""
import argparse,sys

def main():
    p=argparse.ArgumentParser(add_help=False);p.add_argument('--backend',choices=['fastlio2','legacy'],default='fastlio2');args,remaining=p.parse_known_args();sys.argv=[sys.argv[0],*remaining]
    if args.backend=='legacy':
        from mapping_legacy import main as run
    else:
        from mapping_fastlio import main as run
    return run()
if __name__=='__main__':sys.exit(main())
