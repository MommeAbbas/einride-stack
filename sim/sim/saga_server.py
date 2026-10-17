"""Run a compatible external mission app with isolated state and in-memory credentials."""
import argparse
import os
from pathlib import Path
import sys


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--host',default='127.0.0.1'); parser.add_argument('--source',required=True); parser.add_argument('--run-dir',required=True)
    parser.add_argument('--seed',type=int,required=True); parser.add_argument('--port',type=int,required=True)
    args=parser.parse_args()
    sys.dont_write_bytecode=True
    sys.path.insert(0,str(Path(args.source).resolve()))
    from app.credentials import Credentials
    from app.config import Settings
    from app.main import create_app
    import uvicorn
    creds=Credentials(os.environ['AUTONOMY_SIM_ADMIN_USER'],os.environ['AUTONOMY_SIM_ADMIN_PASSWORD'],
                      {i:os.environ[f'AUTONOMY_SIM_TOKEN_{i}'] for i in range(1,5)})
    settings=Settings(route_seed=args.seed,host=args.host,port=args.port,
                      state_file=Path(args.run_dir)/'saga_state.json')
    uvicorn.run(create_app(settings=settings,credentials=creds),host=args.host,port=args.port,
                access_log=False)


if __name__=='__main__':
    main()
