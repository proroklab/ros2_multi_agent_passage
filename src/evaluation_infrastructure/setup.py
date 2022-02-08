from setuptools import setup

package_name = 'evaluation_infrastructure'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jan',
    maintainer_email='jb2270@cam.ac.uk',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "pose_state_server = evaluation_infrastructure.pose_state_server:main",
            "agent_centr_rm_rvo_passage = evaluation_infrastructure.agent_centralized_robomaster_rvo_passage:main",
        ],
    },
)
