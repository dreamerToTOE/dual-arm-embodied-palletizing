"""Hash-bound reuse of G1-1's independent USD-frame mathematical FK oracle."""
from functools import lru_cache
import hashlib,importlib.util,json
from pathlib import Path


def load_oracle():
    binding=json.loads(Path(__file__).with_name('oracle_binding.json').read_text())
    root=Path(__file__).resolve().parents[3]
    source=root/binding['source']
    if hashlib.sha256(source.read_bytes()).hexdigest()!=binding['sha256']:
        raise ValueError('INDEPENDENT_FK_ORACLE_HASH_MISMATCH')
    spec=importlib.util.spec_from_file_location('_task28_b0_usd_oracle',source)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    # Cache immutable, already hash-verified fixture JSON. Equations unchanged.
    module.frozen_data=lru_cache(maxsize=1)(module.frozen_data)
    return module
