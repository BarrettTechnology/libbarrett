This folder contains old versions of the configuration files, listed by date and endpoint type. If your robot is older than the most recent date, copy the configuration files from the appropriate folder to proficio_sandbox/configuration.

Note: As of 10/14/2015, some folders do not contain gravitycal.conf files because they are not available at this time.  In this case, you will need to run bt-wam-gravitycal yourself after running leftConfig or rightConfig. To avoid needing to re-run bt-wam-gravitycal next time you use that configuration, do one the following (depending on configuration) after running bt-wam-gravitycal:
	cp /etc/barrett/calibration_data/wam3/gravitycal.conf ~/libbarrett/proficio_sandbox/configurations/gravitycal.conf.right
	cp /etc/barrett/calibration_data/wam3/gravitycal.conf ~/libbarrett/proficio_sandbox/configurations/gravitycal.conf.left
(Note that if you installed libbarrett to a directory other than ~/libbarrett, you will need to change the path accordingly.)
