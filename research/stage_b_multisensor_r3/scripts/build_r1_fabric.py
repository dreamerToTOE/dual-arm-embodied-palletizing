#!/usr/bin/env python3
"""Bind the installed C++ RationalTime; no SDK modification or time conversion."""
import argparse,hashlib,json,subprocess,sysconfig
from pathlib import Path

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--isaac-root',type=Path,required=True);a=ap.parse_args()
    r3=Path(__file__).resolve().parents[1];build=r3/'native/build';build.mkdir(exist_ok=True)
    source=r3/'native/r1_fabric.cpp';header=a.isaac_root/'kit/dev/fabric/include/omni/fabric/core/RationalTime.h'
    # Match the installed CoreNodes __pybind11_internals_v4_*_cxxabi1011__ ABI.
    command=['g++','-O2','-shared','-std=c++17','-fPIC','-fabi-version=11','-I/usr/include/pybind11','-I'+sysconfig.get_path('include'),'-I'+str(a.isaac_root/'kit/dev/include'),'-I'+str(a.isaac_root/'kit/dev/fabric/include'),str(source),'-o',str(build/('_r1_fabric'+sysconfig.get_config_var('EXT_SUFFIX')))]
    result=subprocess.run(command,capture_output=True,text=True)
    receipt={'command':command,'returncode':result.returncode,'stderr':result.stderr,'source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),'installed_header_sha256':hashlib.sha256(header.read_bytes()).hexdigest()}
    (build/'build_manifest.json').write_text(json.dumps(receipt,indent=2)+'\n')
    assert result.returncode==0,result.stderr
    print(json.dumps(receipt,indent=2))

if __name__=='__main__':main()
