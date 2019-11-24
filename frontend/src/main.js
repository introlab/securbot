/* eslint-disable import/order */
/**
 * This file is the main process call for the UI, it loads the main vue component into the index,
 * set Vue to use bootstrap toolkit for vue and set Vue to use Vue-router for routing of pages and
 * components. The index.js file in router folder describes the routing used.
 * @module main
 * @author Edouard Legare <edouard.legare@usherbrooke.ca>
 */

import 'bootstrap/dist/css/bootstrap.css';
import 'bootstrap-vue/dist/bootstrap-vue.css';
import BootstrapVue from 'bootstrap-vue';

import Vue from 'vue';
import App from './App';
import router from './router';
import store from './store';

import { library } from '@fortawesome/fontawesome-svg-core';
import {
  faCheck,
  faTrash,
  faExchangeAlt,
  faEye,
  faEyeSlash,
  faGamepad,
  faKeyboard,
  faPlus,
  faMinus,
  faMapMarker,
  faPlug,
  faFileImage,
  faExclamationCircle,
} from '@fortawesome/free-solid-svg-icons';
import { FontAwesomeIcon } from '@fortawesome/vue-fontawesome';

library.add(
  faCheck,
  faTrash,
  faExchangeAlt,
  faEye,
  faEyeSlash,
  faGamepad,
  faKeyboard,
  faPlus,
  faMinus,
  faMapMarker,
  faPlug,
  faFileImage,
  faExclamationCircle,
);

Vue.component('font-awesome-icon', FontAwesomeIcon);

Vue.use(BootstrapVue);
Vue.config.productionTip = false;
Vue.config.devtools = true;

new Vue({
  router,
  store,
  render: h => h(App),
}).$mount('#app');
