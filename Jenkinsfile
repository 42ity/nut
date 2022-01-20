@Library('etn-ipm2-jenkins') _

import params.ZprojectPipelineParams
ZprojectPipelineParams parameters = new ZprojectPipelineParams()
//parameters.debugBuildRunTests = false
//parameters.debugBuildRunMemcheck = false
parameters.enableBaseCheck = false
parameters.enableMemCheck = false
//parameters.enableDistCheck = false
parameters.enableInstall = false
parameters.enableCoverity = false
//parameters.requireGoodGitignore = false

etn_ipm2_build_and_tests_pipeline_zproject(parameters)

// Just this missing. Should be upstream part of make check!
//                stage ('nut-driver-enumerator-test') {
//                    when { expression { return ( params.DO_TEST_NDE ) } }
// ...