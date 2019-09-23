<template>
  <!-- App -->
  <div
    id="app"
    class="h-100"
  >
    <!-- Router View to Layout -->
    <router-view />
  </div>
</template>

<script>
import { mapState } from 'vuex';
/**
 * This is the main Vue component used to call all the other using the router-view element.
 *
 * @module App
*/

export default {
  name: 'app',
  computed: mapState({
    darkMode: state => state.darkMode,
  }),
};
</script>

<style>
</style>
