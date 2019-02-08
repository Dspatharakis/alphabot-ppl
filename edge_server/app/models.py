#!flask/bin/python
from flask import Flask
from flask_sqlalchemy import SQLAlchemy
from flask_migrate import Migrate
from config import Config


app = Flask(__name__)
app.config.from_object(Config)

db=SQLAlchemy(app)
migrate = Migrate(app,db)

class Path(db.Model):
    id = db.Column(db.Integer, primary_key=True)
    source = db.Column(db.String(64))#,unique=True, nullable=False)
    path =  db.Column(db.String(128))
    cost =  db.Column(db.Integer()) 
    
    #def __init__(self, title, body):
    #    self.title = title
    #    self.body = body
    #def __repr__(self):
    #    return (self.path,self.cost)
