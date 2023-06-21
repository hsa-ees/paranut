#  -----------------------------------------------------------------------------
#
#  This file is part of the ParaNut project.
#
#  Copyright (C) 2022-2023 Daniel Bortkevych  <daniel.bortkevych@hs-augsburg.de>
#                          Oleg Murashko      <oleg.murashko1@hs-augsburg.de>
#                          Haris Vojic        <haris.vojic@hs-augsburg.de>
#      Hochschule Augsburg, University of Applied Sciences
#
#  Description:
#  	This python file contains an unittest test case, which checks the versions of
#	programs needed by the pn-config-creator tool.
#
#  --------------------- LICENSE -----------------------------------------------
#  Redistribution and use in source and binary forms, with or without 
#  modification, are permitted provided that the following conditions are met:
#
#  1. Redistributions of source code must retain the above copyright notice, 
#     this list of conditions and the following disclaimer.
#
#  2. Redistributions in binary form must reproduce the above copyright notice,
#     this list of conditions and the following disclaimer in the documentation 
#     and/or other materials provided with the distribution.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
#  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
#  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
#  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
#  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
#  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
#  SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND 
#  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
#  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#  -----------------------------------------------------------------------------

# Imports
#  -----------------------------------------------------------------------------
import unittest
import subprocess
import platform

from packaging import version

## Class that verifies the version compability
class VersionTestCase(unittest.TestCase):
	
	## Constructor function
	def setUp(self):
		self.py_version		= "3.8.0"
		self.gtk_version	= "3.24.0"

	## Test function checking the python version
	def test_python_version(self):
		py_version = platform.python_version()
		self.assertTrue(version.parse(py_version) > version.parse(self.py_version))

	## Test function checking the gtk version
	def test_gtk_version(self):
		command = ['gtk-launch', '--version']
		sp = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
		gtk_version = str(sp.communicate()[0], 'UTF-8')
		self.assertTrue(version.parse(gtk_version) > version.parse(self.gtk_version))

if __name__ == '__main__':
	unittest.main()
