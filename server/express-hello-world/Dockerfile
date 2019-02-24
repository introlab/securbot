FROM node

# Application directory
WORKDIR /usr/src/app

# Node build files
COPY package*.json ./

RUN npm install

COPY ./index.js ./index.js

EXPOSE 80

CMD ["npm", "start"]
