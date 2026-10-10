#include <bits/stdc++.h>
using namespace std;
int main(){
    ios::sync_with_stdio(false);
    cin.tie(nullptr);
    // Read all input and output it unchanged (echo)
    string line;
    bool first=true;
    while (getline(cin, line)) {
        if (!first) cout << '\n';
        cout << line;
        first=false;
    }
    return 0;
}
