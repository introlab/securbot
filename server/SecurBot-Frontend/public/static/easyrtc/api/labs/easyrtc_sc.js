/* global define, module, require */
/*!
  Script: easyrtc_sc.js

    Patch easyrtc media constraints for sharing screen Chrome using 
    the experimental chromeMediaSource and chromeMediaSourceId
    properties.

  About: License

    Copyright (c) 2016, Priologic Software Inc.
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:

        * Redistributions of source code must retain the above copyright notice,
          this list of conditions and the following disclaimer.
        * Redistributions in binary form must reproduce the above copyright
          notice, this list of conditions and the following disclaimer in the
          documentation and/or other materials provided with the distribution.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
    IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
    ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
    LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
    CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
    SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
    INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
    CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
    ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.
*/

(function (root, factory) {
    if (typeof define === 'function' && define.amd) {
        //RequireJS (AMD) build system
        define(['easyrtc'], factory);
    } else if (typeof module === 'object' && module.exports) {
        //CommonJS build system
        module.exports = factory(require('easyrtc'));
    } else {
        //Vanilla JS, ensure dependencies are loaded correctly
        if (typeof window.easyrtc !== 'object' || !window.easyrtc) {
            throw new Error("easyrtc_sc requires easyrtc");
        }
        root.easyrtc_sc = factory(window.easyrtc);
  }
}(this, function (easyrtc, undefined) {

"use strict";

    var easyrtc_sc;

    var getUserMediaConstraints = easyrtc.getUserMediaConstraints;
    easyrtc.getUserMediaConstraints = function () {

        var constraints = getUserMediaConstraints();

        // Apply screen constraints
        if ( 
            constraints.video &&
                constraints.video.mandatory &&
                    constraints.video.mandatory.chromeMediaSource === 'screen' &&
                        easyrtc_sc.chromeMediaSource === 'desktop'
        ) {
            constraints.video.mandatory.chromeMediaSource = easyrtc_sc.chromeMediaSource;
            constraints.video.mandatory.chromeMediaSourceId = easyrtc_sc.chromeMediaSourceId;
        }

        return constraints;
    };

    /**
     * EasyRTC Sceen Capture api.
     * @note Require EasyRTC Chome Extension (see easyrtc_sc/README.md dir)
     */
    easyrtc_sc = {

        // chromeMedia extension prop
        chromeMedia: null,           // Chrome Extention listener
        chromeMediaExtention: null,  // Chrome Extention availability
        chromeMediaSource: 'screen', // constraint value for video chromeMediaSource 
        chromeMediaSourceId: null,   // constraint value for video chromeMediaSourceId 

        /**
         * Request screenshare stream via EasyRTC chrome extension if available or 
         * fallback on chrome experimental screen share feature then fail if not avaialble.
         * @param {function(Object)} successCallback - will be called with localmedia stream on success. 
         * @param {function(String,String)} errorCallback - is called with an error code and error description.
         *
         * @example
         * easyrtc_sc.getUserScreen(
         *      function () {
         *          console.log('ok', arguments);
         *      },
         *      function (error) {
         *          console.log('error', error);
         *      }
         *  );
         */
        getUserScreen: function (onSuccess, onFailure) {

            function requestUserScreen() {

                easyrtc.enableAudio(false);
                easyrtc.enableVideo(true);
                easyrtc.setScreenCapture();
                
                easyrtc.initMediaSource(
                    onSuccess,
                    onFailure
                );
            }

            function requestUserScreenId() {

                // Handle Permission Denied Errore
                if (easyrtc_sc.chromeMediaSource === 'PermissionDeniedError') {
                    onFailure('PermissionDeniedError');

                } else if (easyrtc_sc.chromeMediaSourceId) {
                    requestUserScreen();

                } else {

                    // Request extension chromeMediaSourceId value
                    if (easyrtc_sc.chromeMediaSourceId === null) {
                        window.postMessage('get-sourceId', '*');
                        easyrtc_sc.chromeMediaSourceId = false;
                    }

                    setTimeout(requestUserScreenId, 1000);
                }
            }


            if (!easyrtc.supportsGetUserMedia()) {

                onFailure("Do not support getUserMedia");

            } else if (easyrtc.localScreenStream) {

                if (onSuccess) {
                	// Make it async anyway
                    setTimeout(function () {
                        onSuccess(easyrtc_sc.localScreenStream);
                    });
                }

            } else {

                easyrtc_sc.isChromeExtensionAvailable(function (isAvailable) {
                    if (isAvailable) {
                        requestUserScreenId();
                    } else {
                        requestUserScreen();
                    }
                });
            }
        },

        /**
         * Install a chrome exention using appid.
         * - Need to be inside a click event
         * - App host need to be whitelisted inside appstore.
         *
         * @param {String} appid - google chrome webstore unique app id.
         * @param {function(Object)} successCallback - will be called with localmedia stream on success. 
         * @param {function(String,String)} errorCallback - is called with an error code and error description.
         *
         * @example
         *
         * document.body.addEventListener('click', function () {
         *     easyrtc_sc.installChromeExtension(
         *          'hpmoipogpkhegoblaoomcmjiijipjnfg',
         *          function () {
         *              console.log('ok', arguments);
         *          },
         *          function (error) {
         *              console.log('error', error);
         *          }
         *      );
         *  });
         */
        installChromeExtension: function (appid, onSucess, onFailure) {

            window.chrome.webstore.install(
                'https://chrome.google.com/webstore/detail/'+ appid, 
                onSucess, 
                onFailure
            );
        },

        /**
         * Check if easyRTC Chrome version is valid and support chrome extension.
         * @return bool true if valid else false.
         */
    	isChrome: function () {
            return window.navigator.webkitGetUserMedia && window.chrome &&
                    window.chrome.webstore && window.chrome.webstore.install;
        },

        /**
         * Check if easyRTC Chrome extension is available.
         * @param {String} callback gets boolean, true if Chrome extension is available.
         */
        isChromeExtensionAvailable: function (callback) {

            callback = callback || function () {};

            if (!easyrtc_sc.isChrome()) {

                callback(false);

            // Init postMessage listener
            } else {

                if (!easyrtc_sc.chromeMedia) {

                    // Save listener for future CG
                    easyrtc_sc.chromeMedia = easyrtc_sc.chromeExtentionListener.bind(easyrtc_sc);
                    window.addEventListener('message', easyrtc_sc.chromeMedia);
                }

                // Check cache
                if (easyrtc_sc.chromeMediaExtention) {
                    callback(true);

                // Send message to extension
                } else {

                    // ask extension if it is available
                   	window.postMessage('is-extension-loaded', '*');

                    // Under 2000 ms may create failure due chrome exention response delay.
                    setTimeout(function () {
                        easyrtc_sc.chromeMediaExtention = easyrtc_sc.chromeMediaSource === 'desktop';
                        callback(easyrtc_sc.chromeMediaExtention);
                    }, 2000); 
                }                
            }
        },

        chromeExtentionListener: function (event) {

            if (event.origin !== window.location.origin) {
                return;
            }

            // "cancel" button is clicked
            if (event.data === 'PermissionDeniedError') {
                easyrtc_sc.chromeMediaSource = 'PermissionDeniedError';
            
            // extension notified his presence
            } else if (event.data === 'extension-loaded') {
                easyrtc_sc.chromeMediaSource = 'desktop';
            
            // extension shared temp sourceId
            } else if (event.data.chromeMediaSourceId) {
                easyrtc_sc.chromeMediaSourceId = event.data.chromeMediaSourceId;

    	        // Remove listener
    	        window.removeEventListener('message', easyrtc_sc.chromeMedia);
                easyrtc_sc.chromeMedia = null;
            }
        }
    };

return easyrtc_sc;

}));
