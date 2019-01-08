/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/** \author Ioan Sucan */

#include <gtest/gtest.h>
#include <cstdlib>

#include <dirent.h>
#include <sys/types.h>
#include <sys/param.h>
#include <sys/stat.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>

#include <iostream>

int runExternalProcess(const std::string &executable, const std::string &args)
{
    return system((executable + " " + args).c_str());
}

TEST(URDF, CorrectFormat)
{
  DIR           *d;
  struct dirent *dir;
  d = opendir( "robots" );
  ASSERT_TRUE(d != NULL);
  while( ( dir = readdir( d ) ) )
  {
    if( strcmp( dir->d_name, "." ) == 0 ||
        strcmp( dir->d_name, ".." ) == 0 )
    {
      continue;
    }
    if( dir->d_type != DT_DIR )
    {
      std::string dir_name = dir->d_name;
      if (dir_name.find(std::string(".urdf.xacro")) == dir_name.size()-11)
      {
        char pwd[MAXPATHLEN];
        getcwd( pwd, MAXPATHLEN );
        printf("\n\ntesting: %s\n",(std::string(pwd)+"/robots/"+dir_name).c_str());
        EXPECT_EQ(0, runExternalProcess("xacro --inorder", std::string(pwd)+"/robots/"+dir_name+" > test/tmp.urdf"));
        EXPECT_EQ(0, runExternalProcess("check_urdf", "test/tmp.urdf"));
        //break;
      }
    }
  }
  closedir( d );
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
