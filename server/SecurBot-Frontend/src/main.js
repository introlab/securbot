/* 
* Author : SecurBot 2019 team
* File : main.js
* Desc :  This file is the main process call for the UI,
*         it loads the main vue component into the index,
*         set Vue to use bootstrap toolkit for vue and set,
*         Vue to use Vue-router for routing of pages and 
*         components. The index.js file in router folder
*         describes the routing used.
* 
*/


import Vue from 'vue'
import App from './App'
import router from './router';

import BootstrapVue from 'bootstrap-vue'

Vue.use(BootstrapVue)
Vue.config.productionTip = false

import 'bootstrap/dist/css/bootstrap.css'
import 'bootstrap-vue/dist/bootstrap-vue.css'

/* eslint-disable no-new */
new Vue({
  el: '#app',
  router,
  components: { App },
  template: '<App/>'
})
