'''
Author: DexterZzz1010 DexterZzz1010@gmail.com
<<<<<<< HEAD
Date: 2024-10-07 18:45:08
LastEditors: DexterZzz1010 DexterZzz1010@gmail.com
LastEditTime: 2024-10-07 19:53:28
=======
Date: 2024-10-03 15:41:56
LastEditors: DexterZzz1010 DexterZzz1010@gmail.com
LastEditTime: 2024-10-07 16:38:20
>>>>>>> perception
FilePath: /catkin_ws/src/Ias-Skills-Handout/setup.py
Description: vivo50

Copyright (c) 2024 by DexterZzz1010@gmail.com, All Rights Reserved. 
'''
## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['project_ias_skills'],
    package_dir={'': 'src'},
)


setup(**setup_args)
