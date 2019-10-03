# Frontend

- [Frontend](#frontend)
  - [Summary](#summary)
  - [Components](#components)
  - [Environment Variables](#environment-variables)
  - [Changelog](#changelog)
  - [Project commands](#project-commands)
    - [Setup the project](#setup-the-project)
    - [Compiles and hot-reloads for development](#compiles-and-hot-reloads-for-development)
    - [Compiles and minifies for production](#compiles-and-minifies-for-production)
    - [Generates the frontend documentation](#generates-the-frontend-documentation)
    - [Lints and fixes files](#lints-and-fixes-files)
    - [Run your tests](#run-your-tests)
    - [Run your end-to-end tests](#run-your-end-to-end-tests)
    - [Run your unit tests](#run-your-unit-tests)

## Summary

The frontend is been develop using the Vue.js framework with bootstrap toolbox and so is web base.
The interface is there to offer an easy way for operators to connect to any robot in services and perform some specific actions on them.

> We currently do not support mobile devices.

## Components

## Environment Variables
The UI, through Vue/webpack, uses environment variables to set come variables used during execution. The variables are set inside a [.env file](https://cli.vuejs.org/guide/mode-and-env.html#modes).

An example of this is shown inside .env.example. You would need to either create a new file in the same format or create a copy of the file and change its values to the ones you desired.s

- VUE&#95_APP&#95_SERVER&#95_URL: The URL of the server to connect to.
- VUE&#95_APP&#95_SERVER&#95_ROOM&#95_NAME: The room name to connect to on the server.

## Changelog
The changelog file shows all changes since v0.1.0. It was created on September 17th 2019, any changes before that date are available through the commits of the got repo.

## Project commands

### Setup the project
```sh
npm install
```

### Compiles and hot-reloads for development
```sh
npm run serve
```

### Compiles and minifies for production
```sh
npm run build
```

### Generates the frontend documentation
```sh
npm run styleguide:build
```

### Lints and fixes files
```sh
npm run lint
```

### Run your tests
```sh
npm run test
```

### Run your end-to-end tests
```sh
npm run test:e2e
```

### Run your unit tests
```sh
npm run test:unit
```
