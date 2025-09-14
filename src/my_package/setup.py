from setuptools import find_packages, setup

package_name = 'my_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    #need to update dat files
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/'+ package_name, ['package.xml']),
         ('share/'+package_name+'/msg',['msg/Ingredients.msg']),
         ('share/'+package_name+'/msg',['msg/Steps.msg']),
         ('share/'+ package_name +'/srv',['srv/DetectIngredients.srv']),
         ('share/'+package_name+'/srv',['srv/GenerateRecipe.srv']),
    ],
    install_requires=['setuptools','ollama',],
    zip_safe=True,
    maintainer='avery',
    maintainer_email='avery@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    #need add entry points i.e the nodes 
    entry_points={
        'console_scripts': [
            'webcamNode=my_package.webcamNode:main',
            'planningNode=my_package.planningNode:main',
            'controlNode=my_package.controlNode:main'    
            'webcamTest=my_package.webcamTest:main'        
        ],
    },
)
