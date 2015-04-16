PROFICIO DEVELOPMENT REQUIREMENTS:

1) Libbarrett 1.2.2
2) 3 DOF Configuration Files (calibration, zerocal, wam3)


To install on a new CF card or computer that needs to run a Proficio the following steps are required:

  #  svn co http://web.barrett.com/svn/libbarrett/trunk ~/libbarrett
  #  cd libbarrett; cmake . && make && sudo make install
  #  svn co http://web.barrett.com/svn/internal/devel/proficio_sandbox ~/proficio_sandbox
  #  cp ~/proficio_sandbox/configurations/bash_aliases.txt ~/.bash_aliases  (if you don't have your own bash shortcuts setup already).

You should be able to go to ANY proficio system and run the following commands from any terminal location:
showAliases - shows all the commands below.
rightConfig - changes W2B transform so that cartesian 0,0,0 is behind center of patient. corrects home position and gravitycal file
leftConfig - changes W2B transform so that cartesian 0,0,0 is behind center of patient. corrects home position and gravitycal file

This will become obsolete when the Proficio can self-diagnose Left and Right Arm setups. 
