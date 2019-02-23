FROM node

# Application directory
WORKDIR /usr/src/app

# Node build files
COPY package*.json .

RUN npm install

COPY . .

EXPOSE 8080

CMD ["npm", "start"]
