from flask import *
# Create instance of app
app = Flask(__name__)
# Set default motor speeds
m1val = 0; m2val = 0; pow_setting = 10;

@app.route('/')
@app.route('/<m1val>/<m2val>')
def main_page(m1val,m2val):
    print m1val,m2val
    # Call thrust function on a timer:
    #    thrust(m1val,m2val)
    # Return template using motor values for routing purposes
    return render_template('index.html',m1val=m1val,m2val=m2val)


if __name__ == '__main__':
    app.run(debug=True)
