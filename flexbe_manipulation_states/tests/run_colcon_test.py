# Copyright 2023 Philipp Schillinger,  Christopher Newport University
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Philipp Schillinger,  Christopher Newport University nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


"""
pytest testing for flexbe_manipulation_states
"""

from flexbe_testing.py_tester import PyTester


class TestGenericStates(PyTester):

    def __init__(self, *args, **kwargs):
        """ Initial unit test """
        super().__init__(*args, **kwargs)

    @classmethod
    def setUpClass(cls):

        PyTester._package = "flexbe_manipulation_states"
        PyTester._tests_folder = "tests"

        super().setUpClass()  # Do this last after setting package and tests folder

    # The tests
    def test_get_joints_from_srdf_group_import(self):
        """ invoke pytest test """
        self.run_test("get_joints_from_srdf_group_import")

    def test_get_joint_values_import(self):
        """ invoke pytest test """
        self.run_test("get_joint_values_import")

    def test_moveit_to_joints_import(self):
        """ invoke pytest test """
        self.run_test("moveit_to_joints_import")

    def test_srdf_state_to_moveit_import(self):
        """ invoke pytest test """
        self.run_test("srdf_state_to_moveit_import")

    def test_get_joints_from_srdf_import(self):
        """ invoke pytest test """
        self.run_test("get_joints_from_srdf_import")

    def test_joint_state_to_moveit_import(self):
        """ invoke pytest test """
        self.run_test("joint_state_to_moveit_import")

    def test_get_joint_values_dyn_import(self):
        """ invoke pytest test """
        self.run_test("get_joint_values_dyn_import")

    def test_moveit_to_joints_dyn_import(self):
        """ invoke pytest test """
        self.run_test("moveit_to_joints_dyn_import")

    # def test_srdf_state_to_moveit_execute_trajectory_import(self):
    #     """ invoke pytest test """
    #     self.run_test("srdf_state_to_moveit_execute_trajectory_import")
