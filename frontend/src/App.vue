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
/**
 * This is the main Vue component used to call all the other using the router-view element.
 *
 * @module App
 * @version 1.0.0
*/

export default {
  name: 'app',
  mounted() {
    this.$store.dispatch('database/initLocalData');
  },
};
</script>

<style>
</style>
