# Purpose: bind model output indices to semantic labels and reject mappings for a different binary.
import hashlib
import json


# ***
# 函数名字：load_model_labels
# 函数作用：加载类别目录及可选模型绑定表，验证二进制哈希后返回按输出索引排列的业务类别。
# 参数意义与范围：configuration：项目Configuration对象，提供path解析；settings：模型配置，包含classes、file及可选class_map；verify_model：布尔值，True验证当前二进制SHA256。
# ***
def load_model_labels(configuration, settings, verify_model=True):
    catalog = configuration.path(settings['classes']).read_text(encoding='utf-8-sig').splitlines()
    if not catalog or any(not name or name!=name.strip() for name in catalog) or len(set(catalog))!=len(catalog):
        raise ValueError('MODEL_CLASSES_INVALID')
    filename = settings.get('class_map')
    if filename is None:
        return tuple(catalog)
    if not isinstance(filename,str) or not filename:
        raise ValueError('MODEL_CLASS_MAP_PATH_INVALID')
    mapping = json.loads(configuration.path(filename).read_text(encoding='utf-8-sig'))
    if not isinstance(mapping,dict) or type(mapping.get('schema_version')) is not int or mapping['schema_version']!=1:
        raise ValueError('MODEL_CLASS_MAP_SCHEMA_INVALID')
    expected_hash = mapping.get('model_sha256')
    if (not isinstance(expected_hash,str) or len(expected_hash)!=64
            or any(character not in '0123456789abcdef' for character in expected_hash)):
        raise ValueError('MODEL_CLASS_MAP_HASH_INVALID')
    outputs = mapping.get('outputs')
    if not isinstance(outputs,list) or len(outputs)!=len(catalog):
        raise ValueError('MODEL_CLASS_MAP_COUNT_MISMATCH')
    labels = [None]*len(catalog)
    sources = set()
    for entry in outputs:
        if not isinstance(entry,dict):
            raise ValueError('MODEL_CLASS_MAP_ENTRY_INVALID')
        index,label,source = entry.get('index'),entry.get('label'),entry.get('source_name')
        if (type(index) is not int or not 0<=index<len(labels) or labels[index] is not None
                or not isinstance(label,str) or label not in catalog
                or not isinstance(source,str) or not source.strip() or source in sources):
            raise ValueError('MODEL_CLASS_MAP_ENTRY_INVALID')
        labels[index] = label
        sources.add(source)
    if set(labels)!=set(catalog):
        raise ValueError('MODEL_CLASS_MAP_LABELS_MISMATCH')
    if verify_model:
        actual_hash = hashlib.sha256(configuration.path(settings['file']).read_bytes()).hexdigest()
        if actual_hash!=expected_hash:
            raise ValueError('MODEL_CLASS_MAP_HASH_MISMATCH')
    return tuple(labels)
