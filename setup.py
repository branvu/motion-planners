from setuptools import setup, find_packages

setup(
    name='motion_planners',  # Replace with your package name
    version='0.1.0',  # Replace with your package version
    packages=find_packages(include=['motion_planners', 'motion_planners.*']),
    install_requires=[
    ],
    python_requires='>=3.7',  # Specify the minimum Python version required
    description='',
    long_description=open('README.md').read(),
    long_description_content_type='text/markdown',
    author='Caelan Garrett',
)
