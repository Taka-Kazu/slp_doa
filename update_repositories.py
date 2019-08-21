#!/usr/bin/env python

import yaml
import subprocess
import os

if __name__=='__main__':
    dir_name = os.path.dirname(os.path.abspath(__file__))
    print dir_name
    file_name = dir_name + '/dependency.repos'
    repos = dict()
    with open(file_name, 'r') as f:
        repos = yaml.load(f)

    for repository_name in repos['repositories']:
        repository = repos['repositories'][repository_name]
        print repository
        if repository['type'] == 'git':
            repo_dir_name = dir_name + '/' + os.path.basename('./' + repository_name)
            if not os.path.exists(repo_dir_name):
                print 'git', 'clone', '-b', repository['version'], repository['url'], '--depth', '1', repo_dir_name
                try:
                    result = subprocess.check_output(['git', 'clone', '-b', repository['version'], repository['url'], '--depth', '1', repo_dir_name])
                except subprocess.CalledProcessError as e:
                    print e
                    exit(-1)
            else:
                print 'cd', repo_dir_name, '&&', 'git', 'pull', 'origin', repository['version']
                try:
                    os.chdir(repo_dir_name)
                    result = subprocess.check_output(['git', 'pull', 'origin', repository['version']])
                    os.chdir(dir_name)
                except subprocess.CalledProcessError as e:
                    print e
                    exit(-1)
            print ''
    print 'repositories have been successfully updated'
