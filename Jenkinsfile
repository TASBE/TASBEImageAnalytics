#!/usr/bin/env groovy

pipeline {

    agent any

    stages {
        stage('Init') { // Setup dependencies
            steps {
                echo "NODE_NAME = ${env.NODE_NAME}"
				dir('testRepo') {
					git url: 'git@superior.bbn.com:ELM-test'
					if (fileExists('build')) {
						dir('build') {
				            deleteDir
						}
					}
				}                
                dir ('build') // dir cmd creates dir if it doesn't exist
                // Extract Fiji
                unarchive mapping: ['../extlib/fiji-linux64.zip': './']
            }
        }

        // No build stage at this time, as everything is scripts

        stage('Test') {
            steps {
                echo "Beginning tests..."
	            dir('testRepo') {
		            timestamps {
		            	timeout(time: 2, unit: 'HOURS') {
		            	    dir ('tests')
		            	    sh "./cellStatsTest.sh"
		            	} // timeout
		            } // timestamps
				} // dir
            } // steps
        } // stage build & test
                
    } // stages
        
    post {
        always {
                    
            emailext recipientProviders: [[$class: 'DevelopersRecipientProvider'], [$class: 'CulpritsRecipientProvider']], 
                    to: 'elm-team-commits@rlist.app.ray.com',
                    subject: '$DEFAULT_SUBJECT', 
                    body: '''${PROJECT_NAME} - Build # ${BUILD_NUMBER} - ${BUILD_STATUS}

Changes:
${CHANGES}

Failed Tests:
${FAILED_TESTS, onlyRegressions=false}

Check console output at ${BUILD_URL} to view the full results.

Tail of Log:
${BUILD_LOG, maxLines=50}

'''

        } // always
    } // post

} // pipeline
