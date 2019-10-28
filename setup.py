import setuptools

setuptools.setup(
    name='pypogs',
    version='0.1',
    author='Gustav Pettersson @ ESA',
    author_email='gustavpettersson@live.com',
    description='An automated closed-loop satellite tracker for portable telescopes.',
    url='https://github.com/esa/pypogs',
    packages=setuptools.find_packages(),
    install_requires=['numpy >= 1.17.0',
                      'scipy >= 1.3.1',
                      'Pillow >= 6.2.0',
                      'skyfield >= 1.11',
                      'astropy >= 3.2.1',
                      'pyserial >= 3.4',
                      'tifffile >= 2019.7.26'],
)
