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
#  	This pyhton script contains an unittest test case, which checks the file 
#	structure of the program. A failed test indicates that the program was not
#	installed correctly. 
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

#  Imports
#  -----------------------------------------------------------------------------
import unittest
import sys
import os

try:
	os.chdir(ospath.dirname(sys.argv[0]))
	print(os.getcwd())

except:
	pass

## Class testing the file structure of the porgram directory
class FileStructureTestCase(unittest.TestCase):

	## Constructor function
	def setUp(self):
		self.src_dir 	= '../src'
		self.logger_dir	= '../src/logger'
		self.glade_dir	= '../src/glade'
		self.json_dir	= '../src/json'
		self.json_file	= '../src/json/.default.json'
		self.message	= 'Program folder is missing files'

	## Test function proving the existence of the source directory
	def test_src_dir(self):
		self.assertTrue(os.path.exists(self.src_dir), self.message)

	## Test function proving the existence of the glade directory
	def test_logger_dir(self):
		self.assertTrue(os.path.exists(self.logger_dir), self.message)

	## Test function proving the existence of the glade directory
	def test_glade_dir(self):
		self.assertTrue(os.path.exists(self.glade_dir), self.message)

	## Test function proving the existence of the json directory
	def test_json_dir(self):
		self.assertTrue(os.path.exists(self.json_dir), self.message)

	## Test function proving the existence of the default json file
	def	test_json_file(self):
		self.assertTrue(os.path.isfile(self.json_file), self.message)

if __name__ == '__main__':
	unittest.main()	
