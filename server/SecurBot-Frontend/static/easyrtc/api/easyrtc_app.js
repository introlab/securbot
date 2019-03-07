/* global define, module, require, console */
/*!
  Script: easyrtc_app.js

    Provides support file and data transfer support to easyrtc.

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
            throw new Error("easyrtc_app requires easyrtc");
        }
        root.easyrtc = factory(window.easyrtc);
  }
}(this, function (easyrtc, undefined) {

    "use strict";

    /**
     * This file adds additional methods to Easyrtc for simplifying the 
     * management of video-mediastream assignment.
     * @class Easyrtc_App
     */

    /** @private */
    var autoAddCloseButtons = true;

    /** By default, the easyApp routine sticks a "close" button on top of each caller
     * video object that it manages. Call this function(before calling easyApp) to disable that particular feature.
     * @function
     * @memberOf Easyrtc_App
     * @example
     *    easyrtc.dontAddCloseButtons();
     */
    easyrtc.dontAddCloseButtons = function() {
        autoAddCloseButtons = false;
    };

    /**
     * This is a helper function for the easyApp method. It manages the assignment of video streams
     * to video objects. It assumes
     * @param {String} monitorVideoId is the id of the mirror video tag.
     * @param {Array} videoIds is an array of ids of the caller video tags.
     * @private
     */
    function easyAppBody(monitorVideoId, videoIds) {

        var videoIdsP = videoIds || [],
            numPEOPLE = videoIds.length,
            videoIdToCallerMap = {},
            onCall = null, 
            onHangup = null;

        /**
         * Validates that the video ids correspond to dom objects.
         * @param {String} monitorVideoId
         * @param {Array} videoIds
         * @returns {Boolean}
         * @private
         */
        function validateVideoIds(monitorVideoId, videoIds) {
            var i;
            // verify that video ids were not typos.
            if (monitorVideoId && !document.getElementById(monitorVideoId)) {
                easyrtc.showError(easyrtc.errCodes.DEVELOPER_ERR, "The monitor video id passed to easyApp was bad, saw " + monitorVideoId);
                return false;
            }
    
            for (i in videoIds) {
                if (!videoIds.hasOwnProperty(i)) {
                    continue;
                }
                var name = videoIds[i];
                if (!document.getElementById(name)) {
                    easyrtc.showError(easyrtc.errCodes.DEVELOPER_ERR, "The caller video id '" + name + "' passed to easyApp was bad.");
                    return false;
                }
            }
            return true;
        }


        function getCallerOfVideo(videoObject) {
            return videoIdToCallerMap[videoObject.id];
        }

        function setCallerOfVideo(videoObject, callerEasyrtcId) {
            videoIdToCallerMap[videoObject.id] = callerEasyrtcId;
        }

        function videoIsFree(obj) {
            var caller = getCallerOfVideo(obj);
            return (caller === "" || caller === null || caller === undefined);
        }

        function getIthVideo(i) {
            if (videoIdsP[i]) {
                return document.getElementById(videoIdsP[i]);
            }
            else {
                return null;
            }
        }

        function showVideo(video, stream) {
            easyrtc.setVideoObjectSrc(video, stream);
            if (video.style.visibility) {
                video.style.visibility = 'visible';
            }
        }

        function hideVideo(video) {
            easyrtc.setVideoObjectSrc(video, "");
            video.style.visibility = "hidden";
        }

        if (!validateVideoIds(monitorVideoId, videoIdsP)) {
            throw "bad video element id";
        }

        if (monitorVideoId) {
            document.getElementById(monitorVideoId).muted = "muted";
        }

        easyrtc.addEventListener("roomOccupants", 
            function(eventName, eventData) {
                var i;
                for (i = 0; i < numPEOPLE; i++) {
                    var video = getIthVideo(i);
                    if (!videoIsFree(video)) {
                if( !easyrtc.isPeerInAnyRoom(getCallerOfVideo(video))){
                           if( onHangup ) {
                               onHangup(getCallerOfVideo(video), i);
                           }
                           setCallerOfVideo(video, null);
                        }
                    }
                }
            }
        );

        /** Sets an event handler that gets called when an incoming MediaStream is assigned 
         * to a video object. The name is poorly chosen and reflects a simpler era when you could
         * only have one media stream per peer connection.
         * @function
         * @memberOf Easyrtc_App
         * @param {Function} cb has the signature function(easyrtcid, slot){}
         * @example
         *   easyrtc.setOnCall( function(easyrtcid, slot){
         *      console.log("call with " + easyrtcid + "established");
         *   });
         */
        easyrtc.setOnCall = function(cb) {
            onCall = cb;
        };

        /** Sets an event handler that gets called when a call is ended.
         * it's only purpose (so far) is to support transitions on video elements.
         x     * this function is only defined after easyrtc.easyApp is called.
         * The slot is parameter is the index into the array of video ids.
         * Note: if you call easyrtc.getConnectionCount() from inside your callback
         * it's count will reflect the number of connections before the hangup started.
         * @function
         * @memberOf Easyrtc_App
         * @param {Function} cb has the signature function(easyrtcid, slot){}
         * @example
         *   easyrtc.setOnHangup( function(easyrtcid, slot){
         *      console.log("call with " + easyrtcid + "ended");
         *   });
         */
        easyrtc.setOnHangup = function(cb) {
            onHangup = cb;
        };

        /** 
          * Get the easyrtcid of the ith caller, starting at 0.
          * @function
          * @memberOf Easyrtc_App
          * @param {number} i
          * @returns {String}
          */
        easyrtc.getIthCaller = function(i) {
            if (i < 0 || i >= videoIdsP.length) {
                return null;
            }
            var vid = getIthVideo(i);
            return getCallerOfVideo(vid);
        };

        /** 
          * This is the complement of getIthCaller. Given an easyrtcid,
          * it determines which slot the easyrtc is in.
          * @function
          * @memberOf Easyrtc_App
          * @param {string} easyrtcid 
          * @returns {number} or -1 if the easyrtcid is not a caller.
          */
        easyrtc.getSlotOfCaller = function(easyrtcid) {
            var i;
            for (i = 0; i < numPEOPLE; i++) {
                if (easyrtc.getIthCaller(i) === easyrtcid) {
                    return i;
                }
            }
            return -1; // caller not connected
        };

        easyrtc.setOnStreamClosed(function(caller) {
            var i;
            for (i = 0; i < numPEOPLE; i++) {
                var video = getIthVideo(i);
                if (getCallerOfVideo(video) === caller) {
                    hideVideo(video);
                    setCallerOfVideo(video, "");
                    if (onHangup) {
                        onHangup(caller, i);
                    }
                }
            }
        });

        //
        // Only accept incoming calls if we have a free video object to display
        // them in.
        //
        easyrtc.setAcceptChecker(function(caller, helper) {
            var i;
            for (i = 0; i < numPEOPLE; i++) {
                var video = getIthVideo(i);
                if (videoIsFree(video)) {
                    helper(true);
                    return;
                }
            }
            helper(false);
        });

        easyrtc.setStreamAcceptor(function(caller, stream) {
            var i;
            if (easyrtc.debugPrinter) {
                easyrtc.debugPrinter("stream acceptor called");
            }

            var video;

            for (i = 0; i < numPEOPLE; i++) {
                video = getIthVideo(i);
                if (getCallerOfVideo(video) === caller) {
                    showVideo(video, stream);
                    if (onCall) {
                        onCall(caller, i);
                    }
                    return;
                }
            }

            for (i = 0; i < numPEOPLE; i++) {
                video = getIthVideo(i);
                if (videoIsFree(video)) {
                    setCallerOfVideo(video, caller);
                    if (onCall) {
                        onCall(caller, i);
                    }
                    showVideo(video, stream);
                    return;
                }
            }
            //
            // no empty slots, so drop whatever caller we have in the first slot and use that one.
            //
            video = getIthVideo(0);
            if (video) {
                easyrtc.hangup(getCallerOfVideo(video));
                showVideo(video, stream);
                if (onCall) {
                    onCall(caller, 0);
                }
            }

            setCallerOfVideo(video, caller);
        });

        var addControls, parentDiv, closeButton, i;
        if (autoAddCloseButtons) {

            addControls = function(video) {
                parentDiv = video.parentNode;
                setCallerOfVideo(video, "");
                closeButton = document.createElement("div");
                closeButton.className = "easyrtc_closeButton";
                closeButton.onclick = function() {
                    if (getCallerOfVideo(video)) {
                        easyrtc.hangup(getCallerOfVideo(video));
                        hideVideo(video);
                        setCallerOfVideo(video, "");
                    }
                };
                parentDiv.appendChild(closeButton);
            };

            for (i = 0; i < numPEOPLE; i++) {
                addControls(getIthVideo(i));
            }
        }

        var monitorVideo = null;
        if (easyrtc.videoEnabled && monitorVideoId !== null) {
            monitorVideo = document.getElementById(monitorVideoId);
            if (!monitorVideo) {
                console.error("Programmer error: no object called " + monitorVideoId);
                return;
            }
            monitorVideo.muted = "muted";
            monitorVideo.defaultMuted = true;
        }
    }

    /**
     * Provides a layer on top of the easyrtc.initMediaSource and easyrtc.connect, assign the local media stream to
     * the video object identified by monitorVideoId, assign remote video streams to
     * the video objects identified by videoIds, and then call onReady. One of it's
     * side effects is to add hangup buttons to the remote video objects, buttons
     * that only appear when you hover over them with the mouse cursor. This method will also add the
     * easyrtcMirror class to the monitor video object so that it behaves like a mirror.
     * @function
     * @memberOf Easyrtc_App
     *  @param {String} applicationName - name of the application.
     *  @param {String} monitorVideoId - the id of the video object used for monitoring the local stream.
     *  @param {Array} videoIds - an array of video object ids (strings)
     *  @param {Function} onReady - a callback function used on success. It is called with the easyrtcId this peer is known to the server as.
     *  @param {Function} onFailure - a callback function used on failure (failed to get local media or a connection of the signaling server).
     *  @example
     *     easyrtc.easyApp('multiChat', 'selfVideo', ['remote1', 'remote2', 'remote3'],
     *              function(easyrtcId){
     *                  console.log("successfully connected, I am " + easyrtcId);
     *              },
     *              function(errorCode, errorText){
     *                  console.log(errorText);
     *              });
     */
    easyrtc.easyApp = function(applicationName, monitorVideoId, videoIds, onReady, onFailure) {
        
        var gotMediaCallback = null,
            gotConnectionCallback = null;

        easyAppBody(monitorVideoId, videoIds);

        easyrtc.setGotMedia = function(gotMediaCB) {
            gotMediaCallback = gotMediaCB;
        };

        //
        // try to restablish broken connections that weren't caused by a hangup
        //
        easyrtc.setPeerClosedListener( function(easyrtcid) {
           setTimeout( function() {
               if( easyrtc.getSlotOfCaller(easyrtcid)  >= 0 && easyrtc.isPeerInAnyRoom(easyrtcid)) {
                    easyrtc.call(easyrtcid, function(){}, function() {}, function(){});
               }
           }, 1000);
        });

        /** Sets an event handler that gets called when a connection to the signaling
         * server has or has not been made. Can only be called after calling easyrtc.easyApp.
         * @function
         * @memberOf Easyrtc_App
         * @param {Function} gotConnectionCB has the signature (gotConnection, errorText)
         * @example
         *    easyrtc.setGotConnection( function(gotConnection, errorText){
         *        if( gotConnection ){
         *            console.log("Successfully connected to signaling server");
         *        }
         *        else{
         *            console.log("Failed to connect to signaling server because: " + errorText);
         *        }
         *    });
         */
        easyrtc.setGotConnection = function(gotConnectionCB) {
            gotConnectionCallback = gotConnectionCB;
        };
        
        function nextInitializationStep(/* token */) {
            if (gotConnectionCallback) {
                gotConnectionCallback(true, "");
            }
            onReady(easyrtc.myEasyrtcid);
        }

        function postGetUserMedia() {
            if (gotMediaCallback) {
                gotMediaCallback(true, null);
            }
            if (monitorVideoId !== null) {
                easyrtc.setVideoObjectSrc(document.getElementById(monitorVideoId), easyrtc.getLocalStream());
            }
            function connectError(errorCode, errorText) {
                if (gotConnectionCallback) {
                    gotConnectionCallback(false, errorText);
                }
                else if (onFailure) {
                    onFailure(easyrtc.errCodes.CONNECT_ERR, errorText);
                }
                else {
                    easyrtc.showError(easyrtc.errCodes.CONNECT_ERR, errorText);
                }
            }

            easyrtc.connect(applicationName, nextInitializationStep, connectError);
        }

        var stream = easyrtc.getLocalStream(null);
        if (stream) {
            postGetUserMedia();
        }
        else {
            easyrtc.initMediaSource(
                    postGetUserMedia,
                    function(errorCode, errorText) {
                        if (gotMediaCallback) {
                            gotMediaCallback(false, errorText);
                        }
                        else if (onFailure) {
                            onFailure(easyrtc.errCodes.MEDIA_ERR, errorText);
                        }
                        else {
                            easyrtc.showError(easyrtc.errCodes.MEDIA_ERR, errorText);
                        }
                    },
                    null // default stream
                );
        }
    };

return easyrtc;

})); // end of module wrapper
