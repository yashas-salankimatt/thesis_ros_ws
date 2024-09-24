from setuptools import find_packages, setup

package_name = 'dimensional'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/planning.launch.py']),
        ('share/' + package_name + '/launch', ['launch/sim_moveit.launch.py']),
        ('share/' + package_name + '/config', ['config/planning_config.yaml']),
        # ('share/' + package_name + '/config/xarm6', ['config/xarm6/fake_controllers.yaml']),
        # ('share/' + package_name + '/srdf', ['srdf/xarm.srdf.xacro']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yashas',
    maintainer_email='yashas.salankimatt@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'point_cloud_creator = dimensional.point_cloud_creator:main',
            'sim_cam_tf_pub = dimensional.sim_cam_tf_pub:main',
            'grasp_pipeline_test = dimensional.grasp_pipeline_test:main',
            'pose_publisher = dimensional.pose_publisher:main',
            'object_processor = dimensional.object_processor:main',
            'pose_pub_test = dimensional.pose_pub_test:main',
            'gripper_action_client = dimensional.gripper_action_client:main',
        ],
    },
)
