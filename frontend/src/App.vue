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
 * This is the main Vue component use to call all the other using the router-view element.
 *
 * @module App
*/

export default {
  name: 'app',
};
</script>

<style>
</style>
