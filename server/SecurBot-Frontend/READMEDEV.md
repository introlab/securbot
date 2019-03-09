# Read Me if you need a bit of help to start developing.

![Alt text](src/static/logo.png)

## Project setup and run
Look at the other readme file [README.md](README.md)

## Information on Vue.js

Vue.js is a progressive framework for building user interfaces.     
The project was created with the vue_cli tool.    

## Content of the App folder
The application folder contains a few files not in a folder and (for now) 2 folders:   

  * build : Config file for webpack   
  * config : More webpack config (plugins)   
  * src : Contains the all the JavaScript and the Vue components.   
  * node_modules: Will be generated when setting up the project and contains all the modules that node.js needs.   
  * static : Contains ressource that won't go through webpack (like images and api)   
  * index.html: Contain the main html (do not modify).   
  * package.json: Contain everything to setup the project (do not modify).   
  * package-lock.json: Something (do not modify).   
  * .gitignore: Everything that git should ignore, should already be correctly set (do not modify).  

Content of the src folder:   

  * components: All the Vue components.   
  * static: contains all static resources (images, API, etc.).   
  * App.vue : Main Vue component.   
  * main.js : Main JavaScript of the application.   


## How does Vue.js work and how are we using it ?
### How does it work
Vue.js is a progressive framework for building user interfaces.
This enables users to create UI with "components". Vue.js uses modern building tools to bundle modules like [Webpack](https://webpack.js.org/).

### The way SecurBot is using Vue
For the securBot project, the Vue application is using [single file components](https://vuejs.org/v2/guide/single-file-components.html). This means that a component file contains the html, JavaScript and css of that component. All components, except the main components, can be found under src/components. To use single file components, Vue uses Webpack and Babel to compile (serve) and build.   
   
To note: For right now, the easyRTC server and the App Vue (UI) are 2 separate entities.

