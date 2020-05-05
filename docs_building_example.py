# NOTE: Make sure all these are in the requirements!!
import os
from os import path
import tempfile
import git
import subprocess
import docker

# 1. Check that the docs_source folder actually exists
if not path.exists('docs_source'):
    raise RuntimeError("docs_source doesn't exist")

# 2. Build the main image

### Do dts magic

# 3. Clone docs-sphinx-builder (in a tmp folder)
dirpath = tempfile.mkdtemp()
print("dirpath; %s" % dirpath)
git.Git(dirpath).clone("https://github.com/duckietown/docs-sphinx-builder.git", depth=1)

for i in os.listdir('docs_source'):
    cmd = 'cp -r ' + os.path.join('docs_source', i) + ' %s/docs-sphinx-builder/docs/source/.' % dirpath
    subprocess.Popen(cmd, shell=True)

# 4. Build docker file

print("Building the documents image")
args = {'ARCH': 'amd64',
        'MAJOR': 'new-docs-test',
        'BASE_IMAGE': 'duckietown/dt-ros-commons',
        'REPO_NAME': 'dt-ros-commons'}

client = docker.from_env()
image, _ = client.images.build(path='%s/docs-sphinx-builder/docs' % dirpath,
                               buildargs=args)

print('image id: %s' % image.id)

container = client.containers.run(image=image.id,
                                  command='bash -c "cp -r /docs docs; cd docs; make html; cp -r build /docs/build"',
                                  volumes={'%s/docs-sphinx-builder/docs' % dirpath: {'bind': '/docs', 'mode': 'rw'}},
                                  detach=True)

exit_code = container.wait()
print("Exit code: ", exit_code)

print("LOGS:")
print(container.logs())

container.remove(v=True)

# 5. Copy the html files from
cmd = 'if [ -d "docs_html" ]; then rm -Rf docs_html; fi; cp -r ' + '%s/docs-sphinx-builder/docs/build/html' % dirpath + " docs_html"
subprocess.Popen(cmd, shell=True)
print("HTML files copied to docs_html")
