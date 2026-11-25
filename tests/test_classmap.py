# Purpose: verify model-index mapping independently of catalog order and reject stale or ambiguous bindings.
import hashlib
import json
import tempfile
import unittest
from pathlib import Path

from car26.camera.classmap import load_model_labels
from car26.config import Configuration

ROOT = Path(__file__).resolve().parents[1]


class ClassMapTests(unittest.TestCase):
    # ***
    # 函数名字：setUp
    # 函数作用：创建两类别人工模型、反序业务目录和独立索引映射，供绑定测试使用。
    # 参数意义与范围：self：当前测试实例。
    # ***
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(dir=ROOT/'artifacts')
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.model = self.root/'model.bin'
        self.model.write_bytes(b'explicit-test-model')
        (self.root/'classes.name').write_text('greenblock\nredblock\n',encoding='utf-8')
        self.mapping = dict(schema_version=1,model_sha256=hashlib.sha256(self.model.read_bytes()).hexdigest(),
                            outputs=[dict(index=0,source_name='red1',label='redblock'),
                                     dict(index=1,source_name='green1',label='greenblock')])
        self.config = Configuration({},self.root)
        self.settings = dict(file='model.bin',classes='classes.name',class_map='mapping.json')

    # ***
    # 函数名字：load
    # 函数作用：保存当前人工映射并调用正式解析器。
    # 参数意义与范围：self：当前测试实例。
    # ***
    def load(self):
        (self.root/'mapping.json').write_text(json.dumps(self.mapping),encoding='utf-8')
        return load_model_labels(self.config,self.settings)

    # ***
    # 函数名字：test_output_order_ignores_catalog_order
    # 函数作用：验证模型索引决定输出类别，业务目录和JSON记录的排列不改变含义。
    # 参数意义与范围：self：当前测试实例。
    # ***
    def test_output_order_ignores_catalog_order(self):
        self.assertEqual(self.load(),('redblock','greenblock'))
        self.mapping['outputs'].reverse()
        self.assertEqual(self.load(),('redblock','greenblock'))

    # ***
    # 函数名字：test_replacing_model_rejects_stale_map
    # 函数作用：验证只替换二进制而不更新映射时拒绝启动。
    # 参数意义与范围：self：当前测试实例。
    # ***
    def test_replacing_model_rejects_stale_map(self):
        self.model.write_bytes(b'different-model')
        with self.assertRaisesRegex(ValueError,'HASH_MISMATCH'):
            self.load()

    # ***
    # 函数名字：test_duplicate_index_or_semantic_label_rejected
    # 函数作用：验证重复索引、重复业务类别、未知类别及缺失项均不能形成有效映射。
    # 参数意义与范围：self：当前测试实例。
    # ***
    def test_duplicate_index_or_semantic_label_rejected(self):
        second = self.mapping['outputs'][1].copy()
        for key,value in (('index',0),('label','redblock'),('label','unknown'),('source_name','red1')):
            self.mapping['outputs'][1] = dict(second,**{key:value})
            with self.assertRaises(ValueError):
                self.load()
        self.mapping['outputs'].pop()
        with self.assertRaisesRegex(ValueError,'COUNT_MISMATCH'):
            self.load()

    # ***
    # 函数名字：test_models_without_binding_remain_compatible
    # 函数作用：验证未配置class_map的障碍模型仍按原类别文件解码。
    # 参数意义与范围：self：当前测试实例。
    # ***
    def test_models_without_binding_remain_compatible(self):
        self.settings.pop('class_map')
        self.assertEqual(load_model_labels(self.config,self.settings),('greenblock','redblock'))

    # ***
    # 函数名字：test_current_manifest_matches_binary_and_binding
    # 函数作用：核对资产清单记录的模型和索引绑定哈希，避免保留与当前文件不符的旧清单。
    # 参数意义与范围：self：当前测试实例。
    # ***
    def test_current_manifest_matches_binary_and_binding(self):
        records = json.loads((ROOT/'assets/manifest.json').read_text(encoding='utf-8'))
        for destination in ('models/yolo11_x5.bin','models/yolo11_x5.classes.json'):
            record = next(row for row in records if row['destination']==destination)
            data = (ROOT/'assets'/destination).read_bytes()
            self.assertEqual(record['sha256'],hashlib.sha256(data).hexdigest())
            self.assertEqual(record['size'],len(data))

    # ***
    # 函数名字：test_current_model_uses_its_provenanced_index_order
    # 函数作用：按已核实模型哈希检查原始输出顺序，防止换模型时沿用另一权重的类别顺序。
    # 参数意义与范围：self：当前ClassMapTests实例，不运行模型或打开设备。
    # ***
    def test_current_model_uses_its_provenanced_index_order(self):
        from car26.config import load_config
        config = load_config(ROOT/'config/board-object.json')
        settings = config.data['models']['object']
        actual_hash = hashlib.sha256(config.path(settings['file']).read_bytes()).hexdigest()
        # Old order is from artifacts/RDK_CJLU_V01/dnn/classes.names beside
        # the identical 0af8... binary; new order is from the 2026-11-14 deployment.
        expected = {
            '0af8e289007e0ba883fdd1a6bae6f6a7ec2f0821f850443d6180cc43167dd112':
                ('targetOne','targetTwo','targetThree','redblock','blackblock',
                 'greenblock','yellowblock','lightblueblock','darkblueblock'),
            'bedffa66d94d8434cade9a42dd3937f135d416857633d632a63a2af71838457f':
                ('redblock','yellowblock','greenblock','blackblock','lightblueblock',
                 'darkblueblock','targetOne','targetTwo','targetThree'),
        }
        self.assertIn(actual_hash,expected,'A new model needs verified output-order provenance')
        self.assertEqual(load_model_labels(config,settings),expected[actual_hash])


if __name__=='__main__':
    unittest.main()
