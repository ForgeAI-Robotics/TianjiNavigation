import argparse,json
import paths
from clearance_override import configure
p=argparse.ArgumentParser();p.add_argument('state',choices=['on','off']);a=p.parse_args();print(json.dumps(configure(a.state=='on'),ensure_ascii=False,indent=2))
