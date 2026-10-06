#!/usr/bin/env python3

from pathlib import Path
import shutil
import zipfile

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT/'submission/Aditya_Pachauri'
PROJECT = OUT/'Simulation_Files/Robotic_Car_Painting_Cell'


def copy(relative, destination=None):
    source = ROOT/relative
    target = PROJECT/(destination or relative)
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, target)


def copy_tree(relative, destination=None, ignore=None):
    source = ROOT/relative
    target = PROJECT/(destination or relative)
    shutil.copytree(source, target, dirs_exist_ok=True,
                    ignore=shutil.ignore_patterns(*(ignore or [])))


if OUT.exists():
    shutil.rmtree(OUT)
PROJECT.mkdir(parents=True)

# Reviewer-facing files: one README, one detailed report, evidence, and licenses.
for name in ('README.md', 'THIRD_PARTY_NOTICES.md'):
    copy(name)
copy('docs/Technical_Report.md')
copy('docs/Demo_Video_Script.md')
copy('results/execution.json')
for name in ('env.sh', 'env.zsh', 'prepare_car.py', 'generate_figures.py'):
    copy(f'scripts/{name}')

# Complete custom package, excluding generated Python caches and unused thumbnails.
copy_tree('src/painting_cell', ignore=['__pycache__', '*.pyc', 'thumbnails'])

# Minimal upstream UR20 description: only files referenced by cell.urdf.xacro.
for name in ('package.xml', 'LICENSE'):
    copy(f'src/ur_description/{name}')
for name in ('joint_limits.yaml', 'default_kinematics.yaml',
             'physical_parameters.yaml', 'visual_parameters.yaml'):
    copy(f'src/ur_description/config/ur20/{name}')
for name in ('ur_macro.xacro', 'inc/ur_common.xacro', 'inc/ur_joint_control.xacro'):
    copy(f'src/ur_description/urdf/{name}')
copy_tree('src/ur_description/meshes/ur20')

# The upstream CMake file installs every robot variant. This reduced package only
# contains UR20, so its install rule is intentionally reduced too.
ur_cmake = PROJECT/'src/ur_description/CMakeLists.txt'
ur_cmake.write_text(
    'cmake_minimum_required(VERSION 3.5)\n'
    'project(ur_description)\n'
    'find_package(ament_cmake REQUIRED)\n'
    'install(DIRECTORY config meshes urdf DESTINATION share/${PROJECT_NAME})\n'
    'ament_package()\n'
)

# Top-level deliverables and the exact folder names requested by the PDF.
shutil.copy2(ROOT/'Technical_Report.pdf', OUT/'Technical_Report.pdf')
shutil.copytree(ROOT/'media/screenshots', OUT/'Screenshots')
(OUT/'Demo_Video').mkdir()
(OUT/'Demo_Video/RECORDING_REQUIRED.txt').write_text(
    'Record the 3–7 minute narrated run using docs/Demo_Video_Script.md, '
    'then place the MP4 in this folder.\n')
(OUT/'GitHub_Link.txt').write_text('Add the public GitHub repository URL here before submission.\n')
(OUT/'Submission_Email.txt').write_text(
    'To: connect@yuktiiai.in\n'
    'Subject: Assessment submission on robotics simulation\n\n'
    'Full Name: Aditya Pachauri\n'
    'College: [ADD]\nEmail: [ADD]\n'
    'Simulation Software: ROS 2 Jazzy, Gazebo Harmonic, MoveIt 2, RViz 2\n'
    'Robot Selected: Universal Robots UR20\n'
    'GitHub Link: [ADD]\nGoogle Drive Link: [ADD]\n')

archive = OUT/'Simulation_Files/Robotic_Car_Painting_Cell.zip'
with zipfile.ZipFile(archive, 'w', zipfile.ZIP_DEFLATED, compresslevel=9) as bundle:
    for path in sorted(PROJECT.rglob('*')):
        if path.is_file():
            bundle.write(path, path.relative_to(PROJECT.parent))

print(f'Created {OUT}')
print(f'Lean project archive: {archive.stat().st_size/1024/1024:.1f} MiB')
