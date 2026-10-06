from setuptools import setup
from pathlib import Path
package_name = 'painting_cell'
data = [('share/ament_index/resource_index/packages', ['resource/painting_cell']),
        ('share/painting_cell', ['package.xml'])]
for folder in ('launch', 'config', 'urdf', 'worlds', 'models'):
    for parent in sorted({f.parent for f in Path(folder).rglob('*') if f.is_file()}):
        data.append(('share/painting_cell/' + str(parent), [str(f) for f in sorted(parent.iterdir()) if f.is_file()]))
setup(name=package_name, version='0.1.0', packages=[package_name], data_files=data,
      install_requires=['setuptools'], zip_safe=True,
      maintainer='Aditya Pachauri', maintainer_email='aditya@example.com',
      description='UR20 car painting simulation and validation', license='Apache-2.0',
      entry_points={'console_scripts': ['painting_demo = painting_cell.demo:main']})
