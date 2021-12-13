import numpy as np
import os
import trimesh
import sys

class ConverterStlToURDF():
  def __init__(self, input_file_name, output_file_name):
    ###############################################################################
    ## IO file to tri/vertices
    ###############################################################################

    base=os.path.basename(input_file_name)
    name = os.path.splitext(base)[0]

    f = open(output_file_name,'w')
    f.write('<?xml version="1.0"?>\n')

    f.write('<robot name="'+name+'">\n')

    hstr = '<link name="body">\n'
    hstr +='  <collision>\n'
    hstr +='    <origin rpy="0 0 0" xyz="0 0 0"/>\n'
    hstr +='    <geometry>\n'
    hstr +='     <mesh filename="'+str(input_file_name)+'"/>\n'
    hstr +='    </geometry>\n'
    hstr +='  </collision>\n'
    hstr +='  <visual>\n'
    hstr +='    <origin rpy="0 0 0" xyz="0 0 0"/>\n'
    hstr +='    <geometry>\n'
    hstr +='      <mesh filename="'+str(input_file_name)+'"/>\n'
    hstr +='    </geometry>\n'
    hstr +='  </visual>\n'
    hstr +='  </link>\n'
    f.write(hstr)

    f.write('</robot>')
    f.close()


if __name__ == '__main__':
  if len(sys.argv) == 2: 
    input_file_name = sys.argv[1]
    base=os.path.basename(input_file_name)
    name = os.path.splitext(base)[0]
    output_file_name = name + ".urdf"
    print(output_file_name)
    ConverterStlToURDF(input_file_name, output_file_name)
