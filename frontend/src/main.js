/* eslint-disable import/order */
/**
 * Main script file of the Vue.js app. Binds the #app element in index.html with the components.
 * Imports and set [router](https://router.vuejs.org/) + [store](https://vuex.vuejs.org/) to the vue main process.
 * Imports bootstrap "general" toolkit + the bootstrap [vue-bootstrap](https://bootstrap-vue.js.org/) components.
 * Imports some icons from the [font awesome](https://fontawesome.com/icons?d=gallery&m=free) free library.
 *
 * @module main
 * @author Edouard Legare <edouard.legare@usherbrooke.ca>
 * @version 1.2.0
 */

import 'bootstrap/dist/css/bootstrap.css';
import 'bootstrap-vue/dist/bootstrap-vue.css';
import BootstrapVue from 'bootstrap-vue';

import Vue from 'vue';
import App from './App';
import router from './router';
import store from './store';

import { library } from '@fortawesome/fontawesome-svg-core';
// We manually import the icons that we needs and only those icons. Allows better webpack bundling.
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
  faMapMarkerAlt,
  faPlug,
  faFileImage,
  faExclamationCircle,
  faRedo,
  faPowerOff,
  faChevronUp,
  faChevronDown,
} from '@fortawesome/free-solid-svg-icons';
import { FontAwesomeIcon } from '@fortawesome/vue-fontawesome';

// To use the icons, we need to add them to the library of the font-awesome component.
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
  faMapMarkerAlt,
  faPlug,
  faFileImage,
  faExclamationCircle,
  faRedo,
  faPowerOff,
  faChevronUp,
  faChevronDown,
);

// Declare the font-awesome component
Vue.component('font-awesome-icon', FontAwesomeIcon);
// Add bootstrap-vue components
Vue.use(BootstrapVue);

// Vue config parameter
Vue.config.productionTip = false; // ?
Vue.config.devtools = true; // Allows vue-devtools (browser extension) to be used

// Declare Vue main component with router and store
new Vue({
  router,
  store,
  render: h => h(App),
}).$mount('#app');
